import os

os.environ.setdefault("MAVLINK20", "1")

import argparse
import threading
import time

from pymavlink import mavutil

MAV = mavutil.mavlink


class Link:
    def __init__(self, device, baud=57600, sysid=255, compid=190):
        self.master = mavutil.mavlink_connection(
            device,
            baud=baud,
            source_system=sysid,
            source_component=compid,
            autoreconnect=True,
        )
        self.tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._hb_thread = None

    @property
    def target(self):
        return self.master.target_system, self.master.target_component

    def wait_vehicle(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.type == MAV.MAV_TYPE_GCS or msg.autopilot == MAV.MAV_AUTOPILOT_INVALID:
                continue
            self.master.target_system = msg.get_srcSystem()
            self.master.target_component = msg.get_srcComponent()
            return msg
        raise TimeoutError("vehicle heartbeat not received")

    def start_heartbeat(self, rate_hz=1.0):
        if self._hb_thread is not None and self._hb_thread.is_alive():
            return
        self._stop.clear()
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, args=(1.0 / rate_hz,), daemon=True
        )
        self._hb_thread.start()

    def _heartbeat_loop(self, period):
        while not self._stop.is_set():
            with self.tx_lock:
                self.master.mav.heartbeat_send(
                    MAV.MAV_TYPE_GCS,
                    MAV.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    MAV.MAV_STATE_ACTIVE,
                )
            self._stop.wait(period)

    def close(self):
        self._stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
        self.master.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=57600)
    args = parser.parse_args()

    link = Link(args.port, args.baud)
    try:
        hb = link.wait_vehicle()
        link.start_heartbeat()
        print(
            f"sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()} "
            f"type={hb.type} mode={mavutil.mode_string_v10(hb)}"
        )
        time.sleep(3)
    finally:
        link.close()


if __name__ == "__main__":
    main()
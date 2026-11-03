"""Stage 1: stream both Muse IMUs and report how they performed.

Run on the Jetson (Changseob's code is Linux-only), from this folder:
    source venv/bin/activate
    python run_imu_test.py --scan              # find IMU and dongle addresses
    python run_imu_test.py --duration 180      # run the test

Ctrl+C stops early and still prints the report. Reports are saved as JSON
in reports/.
"""
import argparse
import os
import time

from imu_report import (RecordingMuseIMUs, StaleTracker, build_report,
                        format_report, save_report)

# ---- FILL THESE IN (python run_imu_test.py --scan shows the values) -------
# address:     each IMU's Bluetooth MAC
# adapter_mac: MAC of the Bluetooth dongle that IMU should use. Both sides
#              may share one; one dongle per IMU usually performs better.
CFG = {
    "muse_frequency": 100,
    "muse_devices": {
        "left":  {"address": "AA:BB:CC:DD:EE:01", "adapter_mac": "11:22:33:44:55:66"},
        "right": {"address": "AA:BB:CC:DD:EE:02", "adapter_mac": "11:22:33:44:55:66"},
    },
}
# ---------------------------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))


def scan():
    """List Bluetooth dongles and nearby named BLE devices."""
    import asyncio
    import subprocess
    from bleak import BleakScanner

    print("Bluetooth dongles (adapter_mac = BD Address):")
    try:
        out = subprocess.run(["hciconfig", "-a"], capture_output=True,
                             text=True, timeout=5).stdout
        for line in out.splitlines():
            if line.startswith("hci") or "BD Address" in line:
                print("  " + line.strip())
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"  hciconfig failed: {exc!r}")
    print("\nScanning for BLE devices for 10 s (address = MAC)...")
    for d in asyncio.run(BleakScanner.discover(timeout=10)):
        if d.name:
            print(f"  {d.address}  {d.name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=180,
                        help="seconds to stream (default: %(default)s)")
    parser.add_argument("--poll-hz", type=float, default=100,
                        help="how often to check for stale data, like the exo "
                             "control loop (default: %(default)s)")
    parser.add_argument("--scan", action="store_true",
                        help="list dongles and nearby IMUs, then exit")
    parser.add_argument("--no-save", action="store_true",
                        help="print the report without saving it")
    args = parser.parse_args()
    if args.scan:
        scan()
        return

    # ---- How the test works ------------------------------------------------
    # Two measurements run at the same time:
    #
    # 1. Packet log (background thread, imu_report.RecordingMuseIMUs._receive)
    #    Every BLE packet that arrives is logged, one entry per sample:
    #      stamps[side]   - IMU clock (ms): when the IMU took the sample
    #      arrivals[side] - computer clock (s): when the packet reached our code
    #    At the end these give:
    #      dropped     - gaps in the IMU timestamps (5020 -> 5050 ms = 2 lost)
    #      delay/late  - how much later than the fastest sample each arrived
    #                    (relative only: the two clocks are not synchronized)
    #      longest gap - longest time with no packets arriving at all
    #
    # 2. Stale check (main thread, the loop below, every 10 ms like the exo)
    #    snapshot() returns valid=False ("stale") if either IMU is
    #    disconnected or its newest sample is older than 3 sample periods
    #    (30 ms at 100 Hz). Age = computer time now - computer time the
    #    newest sample arrived. StaleTracker counts stale episodes and their
    #    total and longest duration: how often the exo would lose IMU input.
    #
    # Order: connect -> arm stale detection -> log for --duration seconds
    #        -> stop logging, disconnect -> build, print and save the report.
    # Packets that arrive during startup are not logged (recording is False).
    # ------------------------------------------------------------------------
    imus = RecordingMuseIMUs(CFG)
    imus.start()                 # connects, verifies the stream; raises on failure
    stale = StaleTracker()
    invalid_before = imus.invalid_packets.copy()
    imus.arm()                   # stale detection on, as when the exo enables torque
    imus.recording = True

    print(f"Recording for {args.duration:g} s (Ctrl+C to stop early)...")
    t0 = next_tick = next_print = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            if imus.error:
                raise RuntimeError(f"Muse stream stopped: {imus.error!r}")
            _, ages, valid = imus.snapshot()
            now = time.monotonic()
            stale.update(valid, now)
            if now >= next_print:
                print(f"{now - t0:6.1f} s   left age {ages['left'] * 1000:6.1f} ms   "
                      f"right age {ages['right'] * 1000:6.1f} ms   "
                      f"{'OK' if valid else 'STALE'}")
                next_print += 1.0
            next_tick += 1.0 / args.poll_hz
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        end = time.monotonic()
        imus.recording = False
        stale.finish(end)
        imus.close()

    report = build_report(imus, end - t0, stale, invalid_before)
    print(format_report(report))
    if not args.no_save:
        print("Saved:", save_report(report, os.path.join(HERE, "reports")))


if __name__ == "__main__":
    main()

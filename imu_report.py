"""IMU performance logging and reporting for the two Muse IMUs.

Builds on Changseob's driver (Header_MuseIMU.py). Used by run_imu_test.py now
(stage 1, standalone) and meant to be reused by the exo's ROS node later
(stage 2), so the exo prints and saves the same report at shutdown.

Report per IMU:
  dropped     IMU timestamps missing between the first and last record
  delay       how much later than the fastest record each one arrived
              (the IMU and host clocks are not synchronized, so this is
              relative delay, not absolute radio latency)
  late        records delayed by more than one sample period
  max gap     longest time with no packet arriving at the host
Report for the pair:
  stale       times the input was unusable for the controller (either IMU's
              newest sample older than muse_stale_timeout_s, or disconnected)
"""
from array import array
import json
import os
import statistics
import time

from Header_MuseIMU import MuseIMUs, RECORD

SIDES = ("left", "right")


class RecordingMuseIMUs(MuseIMUs):
    """Changseob's MuseIMUs plus a log of every record that arrives.

    Logging happens in _receive (once per BLE packet, every record in it),
    not by polling snapshot(): snapshot() only holds the newest sample, so
    polling would report samples as missing that actually arrived.
    """

    def __init__(self, cfg):
        super().__init__(cfg)
        self.recording = False
        self.arrivals = {s: array("d") for s in SIDES}  # host time.monotonic(), s
        self.stamps = {s: array("d") for s in SIDES}    # IMU clock, ms
        self.packets = {s: 0 for s in SIDES}

    def _receive(self, side, packet, scales):
        if self.recording:
            self.packets[side] += 1
            if len(packet) > 8 and not (len(packet) - 8) % RECORD.size:
                now = time.monotonic()
                # Timestamp is the last 6 bytes of each 24-byte record.
                for off in range(8 + 18, len(packet), RECORD.size):
                    self.arrivals[side].append(now)
                    self.stamps[side].append(
                        int.from_bytes(packet[off:off + 6], "little"))
        super()._receive(side, packet, scales)


class StaleTracker:
    """Counts how often, and for how long, the IMU input was unusable.

    Feed it the `valid` flag from MuseIMUs.snapshot() at the control rate.
    """

    def __init__(self):
        self.events = 0
        self.total_s = 0.0
        self.longest_s = 0.0
        self._since = None

    def update(self, valid, now):
        if not valid and self._since is None:
            self._since = now
            self.events += 1
        elif valid and self._since is not None:
            self._end(now)

    def finish(self, now):
        if self._since is not None:
            self._end(now)

    def _end(self, now):
        duration = now - self._since
        self.total_s += duration
        self.longest_s = max(self.longest_s, duration)
        self._since = None


def side_report(arrivals, stamps, period_ms):
    """Drop, delay and gap statistics for one IMU."""
    arrivals, stamps = list(arrivals), list(stamps)
    unique = sorted(set(stamps))
    r = {"records_received": len(stamps),
         "duplicate_records": len(stamps) - len(unique)}
    if len(unique) < 2:
        r["error"] = "too few records for statistics"
        return r

    # Dropped: gaps in the IMU's own timestamps (a 30 ms jump at 100 Hz = 2 lost).
    steps = [max(1, round((b - a) / period_ms)) for a, b in zip(unique, unique[1:])]
    expected = sum(steps) + 1
    dropped = expected - len(unique)
    r.update(expected_records=expected,
             dropped_records=dropped,
             drop_percent=100.0 * dropped / expected,
             longest_drop_run=max(steps) - 1)

    # Delay: fit out the clock offset and drift between IMU and host, then
    # measure each record's arrival relative to the fastest record.
    slope, intercept = statistics.linear_regression(stamps, arrivals)
    residuals = [t - (intercept + slope * s) for s, t in zip(stamps, arrivals)]
    fastest = min(residuals)
    delays = sorted((d - fastest) * 1000.0 for d in residuals)
    late = sum(d > period_ms for d in delays)
    r.update(delay_avg_ms=statistics.fmean(delays),
             delay_p99_ms=delays[int(0.99 * (len(delays) - 1))],
             delay_max_ms=delays[-1],
             late_records=late,
             late_percent=100.0 * late / len(delays))

    # Longest stretch with no packet arriving at the host.
    packets = sorted(set(arrivals))
    r["max_packet_gap_ms"] = max(
        ((b - a) * 1000.0 for a, b in zip(packets, packets[1:])), default=0.0)
    return r


def build_report(imus, duration_s, stale, invalid_before):
    """Collect everything into one dict (printable and JSON-saveable)."""
    period_ms = 1000.0 / imus.frequency
    report = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frequency_hz": imus.frequency,
        "duration_s": duration_s,
        "stale": {
            "timeout_ms": imus.stale_s * 1000.0,
            "events": stale.events,
            "total_s": stale.total_s,
            "longest_s": stale.longest_s,
            "percent_of_run": 100.0 * stale.total_s / duration_s if duration_s else 0.0,
        },
        "imus": {},
    }
    for side in SIDES:
        r = side_report(imus.arrivals[side], imus.stamps[side], period_ms)
        r["address"] = imus.devices[side]["address"]
        r["packets"] = imus.packets[side]
        r["invalid_packets"] = imus.invalid_packets[side] - invalid_before[side]
        r["received_rate_hz"] = r["records_received"] / duration_s if duration_s else 0.0
        battery = imus.battery.get(side)
        r["battery_at_start"] = (
            {"percent": battery[0], "mV": battery[1]} if battery else None)
        report["imus"][side] = r
    return report


def format_report(report):
    """Human-readable version of build_report()'s output."""
    hz = report["frequency_hz"]
    s = report["stale"]
    lines = [
        "",
        "=" * 60,
        f"MUSE IMU PERFORMANCE REPORT  ({report['created']})",
        f"{hz} Hz, {report['duration_s']:.1f} s run",
        "=" * 60,
    ]
    for side, r in report["imus"].items():
        lines.append(f"\n[{side.upper()}]  {r['address']}")
        b = r["battery_at_start"]
        if b:
            lines.append(f"  battery at start  {b['percent']}% ({b['mV'] / 1000:.2f} V)")
        lines.append(f"  received          {r['records_received']} records "
                     f"({r['received_rate_hz']:.1f} Hz), {r['packets']} packets, "
                     f"{r['invalid_packets']} invalid, {r['duplicate_records']} duplicate")
        if "error" in r:
            lines.append(f"  {r['error']}")
            continue
        lines += [
            f"  dropped           {r['dropped_records']} of {r['expected_records']} "
            f"({r['drop_percent']:.3f} %), longest run {r['longest_drop_run']}",
            f"  delay             avg {r['delay_avg_ms']:.2f} ms, "
            f"p99 {r['delay_p99_ms']:.2f} ms, max {r['delay_max_ms']:.2f} ms "
            "(relative to fastest record)",
            f"  {f'late (> {1000 / hz:g} ms)':<18}{r['late_records']} "
            f"({r['late_percent']:.3f} %)",
            f"  longest gap       {r['max_packet_gap_ms']:.1f} ms with no packets",
        ]
    lines += [
        "\n[BOTH]  stale input (unusable for the controller)",
        f"  {s['events']} events, {s['total_s'] * 1000:.0f} ms total "
        f"({s['percent_of_run']:.3f} % of run), longest {s['longest_s'] * 1000:.0f} ms "
        f"(timeout {s['timeout_ms']:.0f} ms)",
        "",
    ]
    return "\n".join(lines)


def save_report(report, folder):
    """Save as JSON in `folder`; returns the file path."""
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime("imu_report_%Y%m%d_%H%M%S.json"))
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    return path

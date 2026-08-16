#!/usr/bin/env python3
"""Convert a raw NMEA log (e.g. pygpsclient's pygpsdata-*.log) into a
Lat,Lon waypoints CSV that waypoint_follower_node can load directly.

Usage:
    python3 nmea_log_to_waypoints.py <input.log> <output.csv> [--min-dist METERS]
"""
import argparse
import csv
import math
import sys

from pynmeagps import NMEAReader


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def extract_fixes(log_path):
    """Yield (lat, lon) for every GGA sentence with a valid fix (quality > 0)."""
    with open(log_path, "rb") as f:
        for raw in f:
            raw = raw.strip()
            if not raw.startswith(b"$") or b"GGA" not in raw:
                continue
            try:
                msg = NMEAReader.parse(raw + b"\r\n", validate=0)
            except Exception:
                continue
            if msg is None or getattr(msg, "quality", 0) in (0, "", None):
                continue
            if msg.lat == "" or msg.lon == "":
                continue
            yield float(msg.lat), float(msg.lon)


def declutter(points, min_dist):
    if not points or min_dist <= 0:
        return points
    kept = [points[0]]
    for lat, lon in points[1:]:
        plat, plon = kept[-1]
        if haversine_m(plat, plon, lat, lon) >= min_dist:
            kept.append((lat, lon))
    return kept


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input_log")
    ap.add_argument("output_csv")
    ap.add_argument(
        "--min-dist",
        type=float,
        default=0.0,
        help="drop points closer than this many meters to the previous kept point",
    )
    args = ap.parse_args()

    points = list(extract_fixes(args.input_log))
    if not points:
        sys.exit(
            f"No valid GNSS fixes (quality > 0) found in {args.input_log}. "
            "The log was likely recorded without a fix (all GGA quality=0)."
        )

    points = declutter(points, args.min_dist)

    with open(args.output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Lat", "Lon"])
        writer.writerows([[f"{lat:.7f}", f"{lon:.7f}"] for lat, lon in points])

    print(f"Wrote {len(points)} waypoints to {args.output_csv}")


if __name__ == "__main__":
    main()

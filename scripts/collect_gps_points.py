#!/usr/bin/env python3
"""
GPS Point Collector v2 - Interactive script for collecting GPS waypoints.

v2 changes (Codex audit fixes):
  - ONLY uses /gnss, never falls back to /fix
  - Stability check: all samples must be within 2m spread
  - Increased sample count to 10
  - Output format uses ID-keyed dict (matches campus_road_network.yaml schema)

Usage:
    1. Start GPS system: make launch-explore-gps
    2. Wait for valid /gnss (gnss_calibration must complete first)
    3. Run: python3 scripts/collect_gps_points.py
    4. Drive to point, press Enter to stamp
    5. Output: ~/frogbot2026/runtime-data/gnss/collected_points.yaml
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import yaml
import os
import math
import time
from datetime import datetime

OUTPUT_DIR = os.path.expanduser("~/frogbot2026/runtime-data/gnss")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "collected_points.yaml")

SAMPLE_COUNT = 10
SAMPLE_TIMEOUT = 30.0
STABILITY_THRESHOLD_M = 2.0  # max spread among samples (meters)


def haversine_m(lat1, lon1, lat2, lon2):
    """Approximate distance in meters between two GPS points."""
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class GPSCollector(Node):
    def __init__(self):
        super().__init__("gps_point_collector")
        self.latest_gnss = None
        # ONLY subscribe to /gnss (v4 coordinate contract)
        self.gnss_sub = self.create_subscription(
            NavSatFix, "/gnss", self._gnss_cb, 10
        )
        self.get_logger().info("Waiting for /gnss (will NOT use /fix)...")

    def _gnss_cb(self, msg):
        self.latest_gnss = msg


def collect_samples(node, n=SAMPLE_COUNT, timeout=SAMPLE_TIMEOUT):
    """Collect n /gnss samples with stability check."""
    samples = []
    start = time.time()
    while len(samples) < n and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=1.0)
        msg = node.latest_gnss
        if msg and msg.status.status >= 0:
            if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
                samples.append((msg.latitude, msg.longitude, msg.altitude))
                node.latest_gnss = None  # consume, wait for next
    if len(samples) < 3:
        return None, "insufficient samples"

    # Stability check: max pairwise distance must be < threshold
    max_spread = 0.0
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            d = haversine_m(samples[i][0], samples[i][1],
                            samples[j][0], samples[j][1])
            max_spread = max(max_spread, d)

    if max_spread > STABILITY_THRESHOLD_M:
        return None, f"unstable: {max_spread:.1f}m spread (limit {STABILITY_THRESHOLD_M}m)"

    avg_lat = sum(s[0] for s in samples) / len(samples)
    avg_lon = sum(s[1] for s in samples) / len(samples)
    avg_alt = sum(s[2] for s in samples) / len(samples)
    return {
        "lat": avg_lat, "lon": avg_lon, "alt": avg_alt,
        "samples": len(samples), "spread_m": round(max_spread, 2)
    }, None


def main():
    rclpy.init()
    node = GPSCollector()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Load existing data (ID-keyed format)
    nodes = {}
    edges = []
    if os.path.exists(OUTPUT_FILE):
        with open(OUTPUT_FILE, "r") as f:
            data = yaml.safe_load(f) or {}
            nodes = data.get("nodes", {})
            edges = data.get("edges", [])
        print(f"Loaded {len(nodes)} existing points from {OUTPUT_FILE}")

    print("=" * 60)
    print("  GPS Point Collector v2")
    print("  Coordinate source: /gnss ONLY (v4 contract)")
    print("  Stability check: {:.0f}m max spread, {} samples".format(
        STABILITY_THRESHOLD_M, SAMPLE_COUNT))
    print("=" * 60)
    print()
    print("Commands:")
    print("  [Enter]  Stamp current GPS position")
    print("  [e]      Add edge between two points")
    print("  [l]      List all collected points and edges")
    print("  [d]      Delete a point by ID")
    print("  [q]      Save and quit")
    print()

    # Wait for /gnss ONLY - no /fix fallback
    print("Waiting for /gnss topic (gnss_calibration must be running)...")
    print("  NOTE: /fix will NOT be used. If /gnss never arrives,")
    print("  check that gnss_calibration has valid offset.\n")
    while True:
        rclpy.spin_once(node, timeout_sec=1.0)
        gnss = node.latest_gnss
        if gnss:
            if gnss.status.status >= 0 and math.isfinite(gnss.latitude):
                print(f"  /gnss: lat={gnss.latitude:.7f}  lon={gnss.longitude:.7f}")
                print("\n/gnss signal acquired! Ready to collect.\n")
                break
            else:
                s = gnss.status.status
                print(f"  /gnss received but status={s} (waiting for valid fix...)")
        else:
            print("  (no /gnss message yet...)")

    next_id = max([int(k) for k in nodes.keys()], default=0) + 1 if nodes else 1

    while True:
        cmd = input(f"\n[Pt {next_id}] Enter=stamp  e=edge  l=list  d=del  q=quit: ").strip().lower()

        if cmd == "q":
            break

        elif cmd == "l":
            if not nodes:
                print("  (no points yet)")
            else:
                print(f"\n  {'ID':<4} {'Name':<20} {'Lat':<13} {'Lon':<13} {'Spread':<8} Dest")
                print("  " + "-" * 66)
                for nid, p in sorted(nodes.items(), key=lambda x: int(x[0])):
                    d = "Y" if p.get("dest") else ""
                    sp = f"{p.get('spread_m', '?')}m"
                    print(f"  {nid:<4} {p['name']:<20} {p['lat']:<13.7f} {p['lon']:<13.7f} {sp:<8} {d}")
            if edges:
                print(f"\n  Edges: {edges}")
            print()

        elif cmd == "e":
            if len(nodes) < 2:
                print("  Need >= 2 points first.")
                continue
            try:
                a = input("  From ID: ").strip()
                b = input("  To ID: ").strip()
                if a not in nodes or b not in nodes:
                    print(f"  Bad ID. Available: {sorted(nodes.keys(), key=int)}")
                    continue
                edge = [int(a), int(b)]
                rev = [int(b), int(a)]
                if edge not in edges and rev not in edges:
                    edges.append(edge)
                    print(f"  Edge: {nodes[a]['name']} <-> {nodes[b]['name']}")
                else:
                    print("  Edge already exists.")
            except (ValueError, KeyError):
                print("  Invalid input.")

        elif cmd == "d":
            did = input("  Delete point ID: ").strip()
            if did in nodes:
                rm = nodes.pop(did)
                did_int = int(did)
                edges = [e for e in edges if did_int not in e]
                print(f"  Deleted: {rm['name']} (ID {did})")
            else:
                print(f"  ID not found. Available: {sorted(nodes.keys(), key=int)}")

        elif cmd == "":
            name = input("  Name: ").strip()
            if not name:
                name = f"point_{next_id}"
            is_dest = input("  Destination? (y/N): ").strip().lower() == "y"

            print(f"  Collecting {SAMPLE_COUNT} samples from /gnss...")
            result, err = collect_samples(node, n=SAMPLE_COUNT, timeout=SAMPLE_TIMEOUT)

            if err:
                print(f"  FAILED: {err}")
                if "unstable" in err:
                    print("  TIP: Wait for GPS to stabilize, then try again.")
                continue

            pid = str(next_id)
            nodes[pid] = {
                "name": name,
                "lat": round(result["lat"], 7),
                "lon": round(result["lon"], 7),
                "alt": round(result["alt"], 1),
                "dest": is_dest,
                "samples": result["samples"],
                "spread_m": result["spread_m"],
                "source": "/gnss",
                "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }

            print(f"\n  Stamped #{pid}: {name}")
            print(f"    ({result['lat']:.7f}, {result['lon']:.7f})")
            print(f"    alt={result['alt']:.1f}m  samples={result['samples']}  spread={result['spread_m']:.2f}m")

            # Auto-suggest edge to previous point
            prev_id = str(next_id - 1)
            if prev_id in nodes:
                yn = input(f"  Connect to '{nodes[prev_id]['name']}' (ID {prev_id})? (Y/n): ").strip().lower()
                if yn != "n":
                    e = [int(prev_id), next_id]
                    if e not in edges and [next_id, int(prev_id)] not in edges:
                        edges.append(e)
                        print(f"  Edge: {nodes[prev_id]['name']} <-> {name}")

            next_id += 1

    # Save in ID-keyed format (matches campus_road_network.yaml schema)
    out = {"nodes": nodes, "edges": edges}
    with open(OUTPUT_FILE, "w") as f:
        yaml.dump(out, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    print(f"\nSaved {len(nodes)} points, {len(edges)} edges to:")
    print(f"  {OUTPUT_FILE}")
    print(f"\nTo convert to campus_road_network.yaml:")
    print(f"  cp {OUTPUT_FILE} src/navigation/gps_waypoint_dispatcher/config/campus_road_network.yaml")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

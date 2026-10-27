#!/usr/bin/env python3
"""
Interactive collector for a brand-new GPS scene bundle.

This tool only uses /fix so that a brand-new scene can be collected before any
scene-specific startup localization exists. It stores all data in one file:

    ~/frogbot2026/runtime-data/gnss/scene_gps_bundle.yaml
"""

from __future__ import annotations

import math
import re
import shutil
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import yaml

OUTPUT_DIR = Path.home() / "frogbot2026/runtime-data" / "gnss"
OUTPUT_FILE = OUTPUT_DIR / "scene_gps_bundle.yaml"
SAMPLE_COUNT = 10
SAMPLE_TIMEOUT_S = 30.0
STABILITY_THRESHOLD_M = 2.0
LONG_EDGE_WARN_M = 20.0
ENGLISH_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")


def now_str() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius_m = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2.0) ** 2
    )
    return radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def prompt_yes_no(prompt: str, default: bool = False) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    raw = input(f"{prompt} {suffix}: ").strip().lower()
    if not raw:
        return default
    return raw in ("y", "yes")


def validate_node_name(name: str, bundle: dict, exclude_id: str | None = None) -> str:
    if not ENGLISH_NAME_RE.fullmatch(name):
        raise ValueError("name must match [A-Za-z0-9_-]+")

    for node_id, node in bundle.get("nodes", {}).items():
        if exclude_id is not None and node_id == exclude_id:
            continue
        if str(node.get("name", "")).lower() == name.lower():
            raise ValueError(f"name '{name}' already exists")

    return name


def ask_valid_name(bundle: dict, label: str, default: str | None = None, exclude_id: str | None = None) -> str:
    while True:
        prompt = f"  {label}"
        if default:
            prompt += f" [default {default}]"
        raw = input(f"{prompt}: ").strip() or (default or "")
        try:
            return validate_node_name(raw, bundle, exclude_id=exclude_id)
        except ValueError as exc:
            print(f"  INVALID: {exc}")


def valid_fix(msg: NavSatFix | None) -> bool:
    if msg is None:
        return False
    if msg.status.status < 0:
        return False
    if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
        return False
    return True


class FixCollector(Node):
    def __init__(self) -> None:
        super().__init__("scene_fix_collector")
        self.latest_fix: NavSatFix | None = None
        self.create_subscription(NavSatFix, "/fix", self._fix_callback, 10)
        self.get_logger().info("Waiting for /fix (raw GNSS) ...")

    def _fix_callback(self, msg: NavSatFix) -> None:
        self.latest_fix = msg


def collect_fix_samples(node: FixCollector, n: int = SAMPLE_COUNT, timeout_s: float = SAMPLE_TIMEOUT_S):
    samples: list[tuple[float, float, float]] = []
    start_time = time.time()

    while len(samples) < n and (time.time() - start_time) < timeout_s:
        rclpy.spin_once(node, timeout_sec=1.0)
        msg = node.latest_fix
        if valid_fix(msg):
            samples.append((msg.latitude, msg.longitude, msg.altitude))
            node.latest_fix = None

    if len(samples) < 3:
        return None, "insufficient valid /fix samples"

    max_spread = 0.0
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            distance = haversine_m(samples[i][0], samples[i][1], samples[j][0], samples[j][1])
            max_spread = max(max_spread, distance)

    if max_spread > STABILITY_THRESHOLD_M:
        return None, f"unstable: {max_spread:.1f}m spread (limit {STABILITY_THRESHOLD_M}m)"

    return {
        "lat": sum(sample[0] for sample in samples) / len(samples),
        "lon": sum(sample[1] for sample in samples) / len(samples),
        "alt": sum(sample[2] for sample in samples) / len(samples),
        "samples": len(samples),
        "spread_m": round(max_spread, 2),
    }, None


def archive_existing_output() -> None:
    if not OUTPUT_FILE.exists():
        return

    backup_name = OUTPUT_FILE.with_name(
        f"scene_gps_bundle_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
    )
    shutil.move(str(OUTPUT_FILE), str(backup_name))
    print(f"已备份旧文件到: {backup_name}")


def make_empty_bundle(scene_name: str) -> dict:
    return {
        "scene_name": scene_name,
        "coordinate_source": "/fix",
        "created_at": now_str(),
        "updated_at": now_str(),
        "sample_count": SAMPLE_COUNT,
        "stability_threshold_m": STABILITY_THRESHOLD_M,
        "long_edge_warn_m": LONG_EDGE_WARN_M,
        "fixed_origin_node_id": None,
        "nodes": {},
        "edges": [],
    }


def next_node_id(bundle: dict) -> int:
    node_ids = [int(node_id) for node_id in bundle.get("nodes", {}).keys()]
    return max(node_ids, default=0) + 1


def normalize_edge(a: int, b: int) -> list[int]:
    return [min(a, b), max(a, b)]


def add_edge(bundle: dict, a: int, b: int) -> bool:
    if a == b:
        print("  不能把一个点连到自己。")
        return False

    if str(a) not in bundle["nodes"] or str(b) not in bundle["nodes"]:
        print("  点 ID 不存在。")
        return False

    edge = normalize_edge(a, b)
    if edge in bundle["edges"]:
        print("  这条边已经存在。")
        return False

    bundle["edges"].append(edge)
    print(f"  已添加边: {bundle['nodes'][str(edge[0])]['name']} <-> {bundle['nodes'][str(edge[1])]['name']}")
    return True


def rebuild_derived_sections(bundle: dict) -> None:
    nodes = bundle.get("nodes", {})
    origin_id = bundle.get("fixed_origin_node_id")
    if origin_id is not None and str(origin_id) in nodes:
        origin_node = nodes[str(origin_id)]
        bundle["fixed_origin"] = {
            "node_id": int(origin_id),
            "name": origin_node["name"],
            "lat": origin_node["lat"],
            "lon": origin_node["lon"],
            "alt": origin_node["alt"],
        }
    else:
        bundle["fixed_origin"] = None

    anchors = []
    destinations = []
    for node_id, node in sorted(nodes.items(), key=lambda item: int(item[0])):
        if node.get("anchor"):
            anchors.append({"node_id": int(node_id), "name": node["name"]})
        if node.get("dest"):
            destinations.append({"node_id": int(node_id), "name": node["name"]})

    bundle["anchors"] = anchors
    bundle["destinations"] = destinations


def autosave(bundle: dict) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    bundle["updated_at"] = now_str()
    rebuild_derived_sections(bundle)
    with open(OUTPUT_FILE, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(bundle, output_file, allow_unicode=True, sort_keys=False)
    print(f"\n[autosave] 已保存到: {OUTPUT_FILE}\n")


def print_summary(bundle: dict) -> None:
    print("=" * 84)
    print(f"场景名: {bundle.get('scene_name', '(unknown)')}")
    print(f"输出文件: {OUTPUT_FILE}")
    print(f"节点数: {len(bundle.get('nodes', {}))}    边数: {len(bundle.get('edges', []))}")

    origin = bundle.get("fixed_origin")
    if origin:
        print(
            f"固定原点: node {origin['node_id']} {origin['name']} "
            f"({origin['lat']:.7f}, {origin['lon']:.7f})"
        )
    else:
        print("固定原点: (未设置)")

    anchors = bundle.get("anchors", [])
    if anchors:
        print("Anchor 点:")
        for anchor in anchors:
            print(f"  node {anchor['node_id']}: {anchor['name']}")
    else:
        print("Anchor 点: (无)")

    destinations = bundle.get("destinations", [])
    if destinations:
        print("目的地:")
        for dest in destinations:
            print(f"  node {dest['node_id']}: {dest['name']}")
    else:
        print("目的地: (无)")
    print("=" * 84)


def list_nodes(bundle: dict) -> None:
    nodes = bundle.get("nodes", {})
    if not nodes:
        print("\n  (当前没有任何点)\n")
        return

    print("\n  ID   Name                 Lat           Lon           Spread   ANCH  DEST  ORIGIN")
    print("  " + "-" * 92)
    origin_id = bundle.get("fixed_origin_node_id")
    for node_id, node in sorted(nodes.items(), key=lambda item: int(item[0])):
        anchor_text = "Y" if node.get("anchor") else "-"
        dest_text = "Y" if node.get("dest") else "-"
        origin_text = "Y" if origin_id is not None and int(node_id) == int(origin_id) else "-"
        spread_text = f"{node.get('spread_m', '?')}m"
        print(
            f"  {node_id:<4} {node['name']:<20} {node['lat']:<13.7f} {node['lon']:<13.7f} "
            f"{spread_text:<8} {anchor_text:<5} {dest_text:<5} {origin_text}"
        )
    print(f"\n  Edges: {bundle.get('edges', [])}\n")


def stamp_node(node: FixCollector, bundle: dict) -> None:
    default_name = f"node_{next_node_id(bundle)}"
    name = ask_valid_name(bundle, "英文点名", default=default_name)
    is_anchor = prompt_yes_no("  这个点是 anchor 吗?", default=False)
    is_dest = prompt_yes_no("  这个点是 destination 吗?", default=False)
    default_origin = bundle.get("fixed_origin_node_id") is None
    set_origin = prompt_yes_no("  设为固定原点?", default=default_origin)

    print(f"  开始采样 {SAMPLE_COUNT} 个 /fix 样本，请保持车辆静止...")
    result, err = collect_fix_samples(node)
    if err:
        print(f"  FAILED: {err}")
        if "unstable" in err:
            print("  TIP: 再等一会儿，让 GPS 稳定后重试。")
        return

    new_id = str(next_node_id(bundle))
    previous_ids = sorted(bundle.get("nodes", {}).keys(), key=int)
    previous_id = previous_ids[-1] if previous_ids else None

    bundle["nodes"][new_id] = {
        "id": int(new_id),
        "name": name,
        "lat": round(result["lat"], 7),
        "lon": round(result["lon"], 7),
        "alt": round(result["alt"], 1),
        "anchor": is_anchor,
        "dest": is_dest,
        "samples": result["samples"],
        "spread_m": result["spread_m"],
        "source": "/fix",
        "time": now_str(),
    }

    if set_origin:
        bundle["fixed_origin_node_id"] = int(new_id)

    autosave(bundle)

    print(f"  已采点 #{new_id}: {name}")
    print(f"    ({result['lat']:.7f}, {result['lon']:.7f})  alt={result['alt']:.1f}m")
    print(f"    anchor={is_anchor}  dest={is_dest}  samples={result['samples']}  spread={result['spread_m']:.2f}m")

    if previous_id is not None:
        distance_to_prev = haversine_m(
            bundle["nodes"][previous_id]["lat"],
            bundle["nodes"][previous_id]["lon"],
            bundle["nodes"][new_id]["lat"],
            bundle["nodes"][new_id]["lon"],
        )
        print(
            f"  与上一个点 #{previous_id} {bundle['nodes'][previous_id]['name']} 的距离: {distance_to_prev:.1f}m"
        )
        if distance_to_prev > LONG_EDGE_WARN_M:
            print(
                f"  WARNING: 这段距离 > {LONG_EDGE_WARN_M:.0f}m。"
                " 当前路网建议补中间点，不要让边太长。"
            )

        if prompt_yes_no(f"  与上一个点 #{previous_id} 自动连边?", default=True):
            if add_edge(bundle, int(previous_id), int(new_id)):
                autosave(bundle)


def update_node(bundle: dict) -> None:
    node_id = input("  要修改的点 ID: ").strip()
    if node_id not in bundle.get("nodes", {}):
        print("  点不存在。")
        return

    node = bundle["nodes"][node_id]
    print(f"  当前名称: {node['name']}")
    raw_name = input("  新名称 (Enter 保持): ").strip()
    if raw_name:
        try:
            node["name"] = validate_node_name(raw_name, bundle, exclude_id=node_id)
        except ValueError as exc:
            print(f"  INVALID: {exc}")
            return

    current_anchor = "Y" if node.get("anchor") else "N"
    raw_anchor = input(f"  Anchor? (y/n/Enter 保持当前={current_anchor}): ").strip().lower()
    if raw_anchor in ("y", "n"):
        node["anchor"] = raw_anchor == "y"

    current_dest = "Y" if node.get("dest") else "N"
    raw_dest = input(f"  Destination? (y/n/Enter 保持当前={current_dest}): ").strip().lower()
    if raw_dest in ("y", "n"):
        node["dest"] = raw_dest == "y"

    is_origin = bundle.get("fixed_origin_node_id") == int(node_id)
    raw_origin = input(f"  设为固定原点? (y/n/Enter 保持当前={'Y' if is_origin else 'N'}): ").strip().lower()
    if raw_origin == "y":
        bundle["fixed_origin_node_id"] = int(node_id)
    elif raw_origin == "n" and is_origin:
        bundle["fixed_origin_node_id"] = None

    autosave(bundle)
    print("  已更新。")


def delete_node(bundle: dict) -> None:
    node_id = input("  删除点 ID: ").strip()
    if node_id not in bundle.get("nodes", {}):
        print("  点不存在。")
        return

    removed = bundle["nodes"].pop(node_id)
    removed_int = int(node_id)
    bundle["edges"] = [edge for edge in bundle.get("edges", []) if removed_int not in edge]
    if bundle.get("fixed_origin_node_id") == removed_int:
        bundle["fixed_origin_node_id"] = None
    autosave(bundle)
    print(f"  已删除点 #{node_id}: {removed['name']}")


def load_or_create_bundle() -> dict | None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if OUTPUT_FILE.exists():
        print(f"发现已有场景文件: {OUTPUT_FILE}")
        choice = input("选择 [r] 继续 / [n] 新建 / [q] 退出: ").strip().lower() or "r"
        if choice == "q":
            return None
        if choice == "n":
            archive_existing_output()
        else:
            with open(OUTPUT_FILE, "r", encoding="utf-8") as existing_file:
                bundle = yaml.safe_load(existing_file) or {}
            bundle.setdefault("nodes", {})
            bundle.setdefault("edges", [])
            bundle.setdefault("fixed_origin_node_id", None)
            return bundle

    default_name = f"scene_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    scene_name = input(f"输入场景名 [默认 {default_name}]: ").strip() or default_name
    bundle = make_empty_bundle(scene_name)
    autosave(bundle)
    return bundle


def wait_for_first_fix(node: FixCollector) -> None:
    print("等待 /fix 有效数据... 请把车停在空旷处。")
    last_report = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=1.0)
        msg = node.latest_fix
        now = time.time()
        if valid_fix(msg):
            print(f"  /fix ready: lat={msg.latitude:.7f} lon={msg.longitude:.7f}")
            print("\n可以开始采集。\n")
            return
        if now - last_report > 3.0:
            if msg is None:
                print("  (还没有收到 /fix 消息)")
            else:
                print(f"  /fix 已收到但 status={msg.status.status}，继续等待有效 fix...")
            last_report = now


def main() -> None:
    rclpy.init()
    node = FixCollector()
    try:
        bundle = load_or_create_bundle()
        if bundle is None:
            return

        print("=" * 84)
        print("  GPS scene bundle 一体化采集脚本")
        print("  数据源: /fix (raw GNSS)")
        print(f"  每点采样: {SAMPLE_COUNT}  稳定性阈值: {STABILITY_THRESHOLD_M:.1f}m")
        print(f"  自动保存: {OUTPUT_FILE}")
        print("  名称规则: 只允许英文 / 数字 / 下划线 / 连字符")
        print("=" * 84)
        print()
        print("命令:")
        print("  [Enter]/n  采普通图点")
        print("  [e]        手工加边")
        print("  [o]        从已有点里选择固定原点")
        print("  [u]        修改已有点的名字 / anchor / destination")
        print("  [l]        列出当前所有点和边")
        print("  [d]        删除一个点")
        print("  [q]        保存并退出")
        print()

        wait_for_first_fix(node)
        print_summary(bundle)

        while True:
            cmd = input("\n[n/e/o/u/l/d/q] > ").strip().lower()
            if cmd in ("", "n"):
                stamp_node(node, bundle)
            elif cmd == "e":
                if len(bundle.get("nodes", {})) < 2:
                    print("  至少先有两个点。")
                    continue
                try:
                    a = int(input("  From ID: ").strip())
                    b = int(input("  To ID: ").strip())
                except ValueError:
                    print("  请输入数字 ID。")
                    continue
                if add_edge(bundle, a, b):
                    autosave(bundle)
            elif cmd == "o":
                node_id = input("  设为固定原点的点 ID: ").strip()
                if node_id not in bundle.get("nodes", {}):
                    print("  点不存在。")
                    continue
                bundle["fixed_origin_node_id"] = int(node_id)
                autosave(bundle)
                print(f"  已将 #{node_id} 设为固定原点。")
            elif cmd == "u":
                update_node(bundle)
            elif cmd == "l":
                print_summary(bundle)
                list_nodes(bundle)
            elif cmd == "d":
                delete_node(bundle)
            elif cmd == "q":
                autosave(bundle)
                print_summary(bundle)
                print("\n采集完成。后续直接运行 scene 编译脚本即可:")
                print(f"  {OUTPUT_FILE}")
                break
            else:
                print("  未知命令。输入 n/e/o/u/l/d/q。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

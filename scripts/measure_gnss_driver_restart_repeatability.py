#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO


EARTH_RADIUS_M = 6371000.0


@dataclass
class RuntimeDeps:
    rclpy: Any
    Node: Any
    NavSatFix: Any
    np: Any
    plt: Any
    GridSpec: Any
    Circle: Any
    yaml: Any


@dataclass
class DriverProcess:
    process: subprocess.Popen[str]
    log_handle: TextIO


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def runtime_root() -> Path:
    env_root = os.environ.get("FYP_RUNTIME_ROOT")
    if env_root:
        return Path(env_root).expanduser()
    return Path.home() / "frogbot2026" / "runtime-data"


def default_output_dir() -> Path:
    return runtime_root() / "gnss" / "startup_repeatability"


def default_params_file() -> Path:
    return repo_root() / "src" / "bringup" / "config" / "master_params.yaml"


def default_route_file() -> Path:
    return runtime_root() / "gnss" / "current_route.yaml"


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure software-level GNSS driver restart repeatability by "
            "restarting nmea_navsat_driver and sampling /fix."
        )
    )
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--restart-wait-s", type=float, default=5.0)
    parser.add_argument("--warmup-s", type=float, default=10.0)
    parser.add_argument("--sample-s", type=float, default=60.0)
    parser.add_argument("--fix-wait-timeout-s", type=float, default=30.0)
    parser.add_argument("--min-valid-samples-per-round", type=int, default=20)
    parser.add_argument("--topic", default="/fix")
    parser.add_argument("--params-file", type=Path, default=default_params_file())
    parser.add_argument("--output-dir", type=Path, default=default_output_dir())
    parser.add_argument("--output-name", default="gnss-startup-repeatability.png")
    parser.add_argument(
        "--reference",
        choices=("centroid", "route-start-ref", "latlon"),
        default="centroid",
        help="Reference used for ENU offsets. Default: centroid of round means.",
    )
    parser.add_argument("--route-file", type=Path, default=default_route_file())
    parser.add_argument("--ref-lat", type=float)
    parser.add_argument("--ref-lon", type=float)
    parser.add_argument(
        "--takeover-existing-driver",
        action="store_true",
        help="Stop an existing nmea_serial_driver before starting the experiment.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the Enter prompt. Intended for short smoke tests only.",
    )
    parser.add_argument(
        "--save-raw",
        action="store_true",
        default=True,
        help="Save CSV/JSON raw data beside the PNG. Enabled by default.",
    )
    parser.add_argument(
        "--no-save-raw",
        dest="save_raw",
        action="store_false",
        help="Disable CSV/JSON raw data output.",
    )
    parser.add_argument(
        "--save-driver-logs",
        action="store_true",
        help="Save per-round nmea_navsat_driver launch output logs.",
    )
    return parser.parse_args()


def preflight() -> RuntimeDeps:
    missing: list[str] = []

    def require(module_name: str) -> Any:
        try:
            return importlib.import_module(module_name)
        except Exception as exc:
            missing.append(f"{module_name}: {type(exc).__name__}: {exc}")
            return None

    np = require("numpy")
    matplotlib = require("matplotlib")
    if matplotlib is not None:
        matplotlib.use("Agg")
    plt = require("matplotlib.pyplot")
    gridspec = require("matplotlib.gridspec")
    patches = require("matplotlib.patches")
    yaml = require("yaml")
    rclpy = require("rclpy")
    rclpy_node = require("rclpy.node")
    sensor_msgs = require("sensor_msgs.msg")

    if missing:
        details = "\n".join(f"  - {item}" for item in missing)
        raise RuntimeError(
            "dependency preflight failed before sampling:\n"
            f"{details}\n"
            "Install the missing ROS/Python packages on Jetson, then rerun."
        )

    return RuntimeDeps(
        rclpy=rclpy,
        Node=rclpy_node.Node,
        NavSatFix=sensor_msgs.NavSatFix,
        np=np,
        plt=plt,
        GridSpec=gridspec.GridSpec,
        Circle=patches.Circle,
        yaml=yaml,
    )


def valid_fix(msg: Any | None) -> bool:
    if msg is None:
        return False
    if msg.status.status < 0:
        return False
    if not math.isfinite(float(msg.latitude)) or not math.isfinite(float(msg.longitude)):
        return False
    return True


def fix_key(msg: Any) -> tuple[int, int, float, float, float, int]:
    alt = float(msg.altitude)
    if not math.isfinite(alt):
        alt = float("nan")
    return (
        int(msg.header.stamp.sec),
        int(msg.header.stamp.nanosec),
        round(float(msg.latitude), 9),
        round(float(msg.longitude), 9),
        round(alt, 3) if math.isfinite(alt) else 0.0,
        int(msg.status.status),
    )


def create_fix_collector(deps: RuntimeDeps, topic: str) -> Any:
    class FixCollector(deps.Node):  # type: ignore[misc, valid-type]
        def __init__(self) -> None:
            super().__init__("gnss_driver_restart_repeatability")
            self.latest_fix: Any | None = None
            self.last_key: tuple[int, int, float, float, float, int] | None = None
            self.create_subscription(deps.NavSatFix, topic, self._callback, 20)

        def _callback(self, msg: Any) -> None:
            self.latest_fix = msg

        def consume_new_valid_fix(self) -> Any | None:
            msg = self.latest_fix
            if not valid_fix(msg):
                return None
            key = fix_key(msg)
            if key == self.last_key:
                return None
            self.last_key = key
            return msg

        def reset_sample_key(self) -> None:
            self.last_key = None

    return FixCollector()


def has_publishers(deps: RuntimeDeps, node: Any, topic: str, settle_s: float = 1.0) -> bool:
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        deps.rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_publishers_info_by_topic(topic):
            return True
    return bool(node.get_publishers_info_by_topic(topic))


def wait_for_no_publishers(
    deps: RuntimeDeps, node: Any, topic: str, timeout_s: float = 5.0
) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        deps.rclpy.spin_once(node, timeout_sec=0.1)
        if not node.get_publishers_info_by_topic(topic):
            return True
    return not bool(node.get_publishers_info_by_topic(topic))


def stop_existing_nmea_driver() -> None:
    if os.name == "nt":
        raise RuntimeError("--takeover-existing-driver is only supported on Linux/Jetson")
    subprocess.run(["pkill", "-INT", "-f", "[n]mea_serial_driver"], check=False)
    time.sleep(2.0)
    still_running = subprocess.run(
        ["pgrep", "-f", "[n]mea_serial_driver"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if still_running.returncode == 0:
        subprocess.run(["pkill", "-KILL", "-f", "[n]mea_serial_driver"], check=False)
        time.sleep(1.0)


def start_gnss_driver(args: argparse.Namespace, round_id: int) -> DriverProcess:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.save_driver_logs:
        log_path = args.output_dir / f"nmea_round_{round_id:02d}.log"
        log_handle = open(log_path, "w", encoding="utf-8")
        internal_log_dir = args.output_dir / "driver_logs"
    else:
        log_handle = open(os.devnull, "w", encoding="utf-8")
        internal_log_dir = args.output_dir / ".driver_logs_tmp"

    env = os.environ.copy()
    env["FYP_LOG_SESSION_DIR"] = str(internal_log_dir)

    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "nmea_navsat_driver",
            "nmea_serial_driver.launch.py",
            f"params_file:={args.params_file.expanduser()}",
        ],
        cwd=str(repo_root()),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=(os.name != "nt"),
        env=env,
    )
    return DriverProcess(process=process, log_handle=log_handle)


def stop_gnss_driver(driver: DriverProcess | None) -> None:
    if driver is None:
        return
    process = driver.process
    try:
        if process.poll() is None:
            if os.name == "nt":
                process.terminate()
            else:
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                if os.name == "nt":
                    process.kill()
                else:
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
    finally:
        driver.log_handle.close()


def wait_for_valid_fix(
    deps: RuntimeDeps,
    node: Any,
    driver: DriverProcess,
    timeout_s: float,
) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if driver.process.poll() is not None:
            raise RuntimeError("nmea_serial_driver exited before publishing a valid /fix")
        deps.rclpy.spin_once(node, timeout_sec=0.2)
        msg = node.consume_new_valid_fix()
        if msg is not None:
            return msg
    raise RuntimeError(f"timed out waiting {timeout_s:.1f}s for valid /fix")


def spin_for_duration(
    deps: RuntimeDeps,
    node: Any,
    driver: DriverProcess,
    duration_s: float,
) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        if driver.process.poll() is not None:
            raise RuntimeError("nmea_serial_driver exited during warm-up")
        deps.rclpy.spin_once(node, timeout_sec=0.2)


def sample_from_msg(round_id: int, msg: Any, received_wall_s: float) -> dict[str, Any]:
    cov = list(msg.position_covariance)
    alt = float(msg.altitude)
    return {
        "round": round_id,
        "wall_time_s": received_wall_s,
        "stamp_sec": int(msg.header.stamp.sec),
        "stamp_nanosec": int(msg.header.stamp.nanosec),
        "lat": float(msg.latitude),
        "lon": float(msg.longitude),
        "alt": alt if math.isfinite(alt) else None,
        "status": int(msg.status.status),
        "cov_x": float(cov[0]) if len(cov) > 0 else None,
        "cov_y": float(cov[4]) if len(cov) > 4 else None,
        "cov_z": float(cov[8]) if len(cov) > 8 else None,
    }


def collect_round(
    deps: RuntimeDeps,
    node: Any,
    driver: DriverProcess,
    args: argparse.Namespace,
    round_id: int,
) -> list[dict[str, Any]]:
    print(f"[round {round_id:02d}] waiting for valid {args.topic} ...", flush=True)
    wait_for_valid_fix(deps, node, driver, args.fix_wait_timeout_s)

    print(f"[round {round_id:02d}] warm-up {args.warmup_s:.1f}s ...", flush=True)
    spin_for_duration(deps, node, driver, args.warmup_s)
    node.reset_sample_key()

    print(f"[round {round_id:02d}] sampling {args.sample_s:.1f}s ...", flush=True)
    samples: list[dict[str, Any]] = []
    deadline = time.monotonic() + args.sample_s
    while time.monotonic() < deadline:
        if driver.process.poll() is not None:
            raise RuntimeError("nmea_serial_driver exited during sampling")
        deps.rclpy.spin_once(node, timeout_sec=0.2)
        msg = node.consume_new_valid_fix()
        if msg is None:
            continue
        samples.append(sample_from_msg(round_id, msg, time.time()))

    if len(samples) < args.min_valid_samples_per_round:
        raise RuntimeError(
            f"round {round_id} collected only {len(samples)} valid samples; "
            f"minimum is {args.min_valid_samples_per_round}"
        )
    return samples


def latlon_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    east = dlon * math.cos(math.radians(ref_lat)) * EARTH_RADIUS_M
    north = dlat * EARTH_RADIUS_M
    return east, north


def max_pairwise_distance(points: list[tuple[float, float]]) -> float:
    max_dist = 0.0
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            max_dist = max(
                max_dist,
                math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1]),
            )
    return max_dist


def load_route_start_ref(deps: RuntimeDeps, route_file: Path) -> tuple[float, float]:
    route_file = route_file.expanduser()
    with open(route_file, "r", encoding="utf-8") as handle:
        data = deps.yaml.safe_load(handle) or {}
    start_ref = data.get("start_ref")
    if not isinstance(start_ref, dict):
        raise RuntimeError(f"route file has no start_ref: {route_file}")
    return float(start_ref["lat"]), float(start_ref["lon"])


def choose_reference(
    deps: RuntimeDeps,
    args: argparse.Namespace,
    round_means: list[dict[str, Any]],
) -> tuple[float, float, str]:
    if args.reference == "centroid":
        ref_lat = sum(item["mean_lat"] for item in round_means) / len(round_means)
        ref_lon = sum(item["mean_lon"] for item in round_means) / len(round_means)
        return ref_lat, ref_lon, "centroid of round means"
    if args.reference == "route-start-ref":
        ref_lat, ref_lon = load_route_start_ref(deps, args.route_file)
        return ref_lat, ref_lon, f"route start_ref: {args.route_file}"
    if args.ref_lat is None or args.ref_lon is None:
        raise RuntimeError("--reference latlon requires --ref-lat and --ref-lon")
    return float(args.ref_lat), float(args.ref_lon), "manual lat/lon reference"


def analyze_samples(
    deps: RuntimeDeps,
    args: argparse.Namespace,
    all_samples: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    np = deps.np
    round_means: list[dict[str, Any]] = []
    for round_id in sorted({int(sample["round"]) for sample in all_samples}):
        samples = [sample for sample in all_samples if int(sample["round"]) == round_id]
        lat_values = np.array([sample["lat"] for sample in samples], dtype=float)
        lon_values = np.array([sample["lon"] for sample in samples], dtype=float)
        alt_values = np.array(
            [sample["alt"] for sample in samples if sample["alt"] is not None],
            dtype=float,
        )
        round_means.append(
            {
                "round": round_id,
                "samples": len(samples),
                "mean_lat": float(np.mean(lat_values)),
                "mean_lon": float(np.mean(lon_values)),
                "mean_alt": float(np.mean(alt_values)) if len(alt_values) else None,
            }
        )

    ref_lat, ref_lon, ref_label = choose_reference(deps, args, round_means)

    for sample in all_samples:
        east, north = latlon_to_enu(sample["lat"], sample["lon"], ref_lat, ref_lon)
        sample["east_m"] = east
        sample["north_m"] = north

    for round_mean in round_means:
        mean_east, mean_north = latlon_to_enu(
            round_mean["mean_lat"], round_mean["mean_lon"], ref_lat, ref_lon
        )
        round_samples = [
            sample for sample in all_samples if int(sample["round"]) == int(round_mean["round"])
        ]
        local_points = [
            latlon_to_enu(
                sample["lat"],
                sample["lon"],
                round_mean["mean_lat"],
                round_mean["mean_lon"],
            )
            for sample in round_samples
        ]
        local_e = np.array([point[0] for point in local_points], dtype=float)
        local_n = np.array([point[1] for point in local_points], dtype=float)
        local_r = np.hypot(local_e, local_n)
        round_mean["east_m"] = mean_east
        round_mean["north_m"] = mean_north
        round_mean["distance_to_ref_m"] = float(math.hypot(mean_east, mean_north))
        round_mean["within_std_e_m"] = float(np.std(local_e))
        round_mean["within_std_n_m"] = float(np.std(local_n))
        round_mean["within_radial_std_m"] = float(
            math.hypot(round_mean["within_std_e_m"], round_mean["within_std_n_m"])
        )
        round_mean["within_max_radius_m"] = float(np.max(local_r)) if len(local_r) else 0.0
        round_mean["within_spread_m"] = max_pairwise_distance(local_points)

    distances = np.array([item["distance_to_ref_m"] for item in round_means], dtype=float)
    radial_stds = np.array([item["within_radial_std_m"] for item in round_means], dtype=float)
    summary = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "topic": args.topic,
        "rounds": len(round_means),
        "sample_s": args.sample_s,
        "warmup_s": args.warmup_s,
        "restart_wait_s": args.restart_wait_s,
        "total_valid_samples": len(all_samples),
        "reference": args.reference,
        "reference_label": ref_label,
        "reference_lat": ref_lat,
        "reference_lon": ref_lon,
        "max_mean_distance_m": float(np.max(distances)),
        "mean_mean_distance_m": float(np.mean(distances)),
        "rms_radius_m": float(math.sqrt(np.mean(distances**2))),
        "median_within_radial_std_m": float(np.median(radial_stds)),
    }
    return round_means, summary


def write_raw_outputs(
    args: argparse.Namespace,
    all_samples: list[dict[str, Any]],
    round_means: list[dict[str, Any]],
    summary: dict[str, Any],
) -> dict[str, Path]:
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = now_stamp()
    samples_path = args.output_dir / f"samples_{stamp}.csv"
    rounds_path = args.output_dir / f"round_summary_{stamp}.csv"
    summary_path = args.output_dir / f"summary_{stamp}.json"

    with open(samples_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_samples[0].keys()))
        writer.writeheader()
        writer.writerows(all_samples)

    with open(rounds_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(round_means[0].keys()))
        writer.writeheader()
        writer.writerows(round_means)

    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return {
        "samples": samples_path,
        "round_summary": rounds_path,
        "summary": summary_path,
    }


def configure_plot_style(deps: RuntimeDeps) -> None:
    deps.plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
            "font.size": 9,
            "axes.linewidth": 0.8,
            "axes.labelsize": 10,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 8,
            "legend.framealpha": 0.9,
            "legend.edgecolor": "0.8",
            "lines.linewidth": 1.0,
            "lines.antialiased": True,
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.08,
        }
    )


def plot_results(
    deps: RuntimeDeps,
    args: argparse.Namespace,
    all_samples: list[dict[str, Any]],
    round_means: list[dict[str, Any]],
    summary: dict[str, Any],
) -> Path:
    configure_plot_style(deps)
    np = deps.np
    plt = deps.plt

    c_samples = "#DBEAFE"
    c_mean = "#2563EB"
    c_ref = "#1E293B"
    c_bar = "#059669"
    c_warn = "#DC2626"
    c_grid = "#CBD5E1"

    fig = plt.figure(figsize=(8.0, 4.8))
    gs = deps.GridSpec(
        1,
        2,
        width_ratios=[1.32, 1.0],
        wspace=0.32,
        top=0.82,
        bottom=0.14,
        left=0.09,
        right=0.96,
    )

    ax_scatter = fig.add_subplot(gs[0])
    ax_bar = fig.add_subplot(gs[1])

    for round_id in sorted({int(sample["round"]) for sample in all_samples}):
        samples = [sample for sample in all_samples if int(sample["round"]) == round_id]
        ax_scatter.scatter(
            [sample["east_m"] for sample in samples],
            [sample["north_m"] for sample in samples],
            s=8,
            color=c_samples,
            alpha=0.45,
            linewidths=0,
        )

    mean_e = np.array([item["east_m"] for item in round_means], dtype=float)
    mean_n = np.array([item["north_m"] for item in round_means], dtype=float)
    ax_scatter.scatter(
        mean_e,
        mean_n,
        s=48,
        color=c_mean,
        edgecolor="white",
        linewidth=0.7,
        label="Round mean",
        zorder=3,
    )
    for item in round_means:
        ax_scatter.annotate(
            str(item["round"]),
            (item["east_m"], item["north_m"]),
            xytext=(4, 3),
            textcoords="offset points",
            fontsize=7,
            color=c_ref,
        )

    ax_scatter.scatter(
        [0.0],
        [0.0],
        s=64,
        marker="+",
        color=c_ref,
        linewidth=1.2,
        label="Reference",
        zorder=4,
    )
    for radius, alpha in ((2.0, 0.55), (5.0, 0.45)):
        ax_scatter.add_patch(
            deps.Circle(
                (0.0, 0.0),
                radius,
                fill=False,
                color=c_ref,
                linestyle="--",
                linewidth=0.7,
                alpha=alpha,
            )
        )
        ax_scatter.text(
            radius,
            0.0,
            f"{radius:.0f}m",
            fontsize=7,
            color=c_ref,
            va="bottom",
            ha="left",
            alpha=0.75,
        )

    all_e = np.array([sample["east_m"] for sample in all_samples] + list(mean_e), dtype=float)
    all_n = np.array([sample["north_m"] for sample in all_samples] + list(mean_n), dtype=float)
    max_extent = max(5.5, float(np.max(np.abs(all_e))), float(np.max(np.abs(all_n)))) + 0.8
    ax_scatter.set_xlim(-max_extent, max_extent)
    ax_scatter.set_ylim(-max_extent, max_extent)
    ax_scatter.set_aspect("equal", adjustable="box")
    ax_scatter.grid(True, color=c_grid, linewidth=0.4, alpha=0.6)
    ax_scatter.set_xlabel("East Offset (m)")
    ax_scatter.set_ylabel("North Offset (m)")
    ax_scatter.set_title("ENU Scatter by Driver Restart", pad=8)
    ax_scatter.legend(loc="upper left")

    rounds = np.array([item["round"] for item in round_means], dtype=int)
    distances = np.array([item["distance_to_ref_m"] for item in round_means], dtype=float)
    bars = ax_bar.bar(rounds, distances, color=c_bar, alpha=0.78, edgecolor="white", linewidth=0.7)
    ax_bar.axhline(
        summary["mean_mean_distance_m"],
        color=c_ref,
        linestyle=":",
        linewidth=0.8,
        label=f"Mean {summary['mean_mean_distance_m']:.2f} m",
    )
    ax_bar.axhline(5.0, color=c_warn, linestyle="--", linewidth=0.7, alpha=0.65, label="5 m")
    for bar, dist in zip(bars, distances):
        ax_bar.annotate(
            f"{dist:.1f}",
            xy=(bar.get_x() + bar.get_width() / 2.0, bar.get_height()),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=7,
            color=c_ref,
        )
    ax_bar.set_xlabel("Restart Round")
    ax_bar.set_ylabel("Mean Distance to Reference (m)")
    ax_bar.set_title("Round Mean Offset", pad=8)
    ax_bar.set_xticks(rounds)
    ax_bar.set_ylim(0.0, max(5.5, float(np.max(distances)) * 1.25))
    ax_bar.grid(True, axis="y", color=c_grid, linewidth=0.4, alpha=0.6)
    ax_bar.legend(loc="upper left")

    fig.suptitle(
        "Software GNSS Driver Restart Repeatability",
        fontsize=12,
        fontweight="bold",
        y=0.96,
    )
    subtitle = (
        f"nmea_navsat_driver -> {args.topic} | "
        f"{summary['rounds']} rounds x {summary['sample_s']:.0f}s | "
        f"reference: {summary['reference_label']}"
    )
    fig.text(0.5, 0.895, subtitle, ha="center", fontsize=8, color="#475569", fontstyle="italic")

    annotation = (
        f"max offset {summary['max_mean_distance_m']:.2f} m\n"
        f"mean offset {summary['mean_mean_distance_m']:.2f} m\n"
        f"RMS radius {summary['rms_radius_m']:.2f} m\n"
        f"median within-round std {summary['median_within_radial_std_m']:.2f} m\n"
        f"valid samples {summary['total_valid_samples']}"
    )
    ax_bar.text(
        0.98,
        3.0,
        annotation,
        transform=ax_bar.get_yaxis_transform(),
        ha="right",
        va="center",
        fontsize=7,
        color=c_ref,
        bbox={"boxstyle": "round,pad=0.28", "facecolor": "white", "edgecolor": "#CBD5E1", "alpha": 0.95},
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_path = args.output_dir / args.output_name
    fig.savefig(output_path, dpi=300, facecolor="white", edgecolor="none")
    plt.close(fig)
    return output_path


def print_header(args: argparse.Namespace) -> None:
    print("=" * 72)
    print("GNSS Driver Restart Repeatability")
    print("=" * 72)
    print("Experiment definition:")
    print("  Software-level GNSS driver restart anchoring variation.")
    print("  GNSS receiver stays powered. Only nmea_serial_driver is restarted.")
    print("")
    print("Parameters:")
    print(f"  rounds:              {args.rounds}")
    print(f"  restart_wait_s:      {args.restart_wait_s}")
    print(f"  warmup_s:            {args.warmup_s}")
    print(f"  sample_s:            {args.sample_s}")
    print(f"  topic:               {args.topic}")
    print(f"  params_file:         {args.params_file.expanduser()}")
    print(f"  output_png:          {args.output_dir / args.output_name}")
    print(f"  reference:           {args.reference}")
    print("=" * 72)


def run_experiment(deps: RuntimeDeps, args: argparse.Namespace) -> tuple[Path, dict[str, Any], dict[str, Path]]:
    args.output_dir = args.output_dir.expanduser()
    args.params_file = args.params_file.expanduser()
    args.route_file = args.route_file.expanduser()

    if not args.params_file.exists():
        raise RuntimeError(f"params file not found: {args.params_file}")
    if args.reference == "route-start-ref" and not args.route_file.exists():
        raise RuntimeError(f"route file not found: {args.route_file}")

    deps.rclpy.init(args=None)
    node = create_fix_collector(deps, args.topic)
    current_driver: DriverProcess | None = None
    all_samples: list[dict[str, Any]] = []
    try:
        if has_publishers(deps, node, args.topic):
            if not args.takeover_existing_driver:
                raise RuntimeError(
                    f"{args.topic} already has a publisher. Stop the running system first:\n"
                    "  cd ~/frogbot2026 && make kill-runtime\n"
                    "Then rerun this script. Use --takeover-existing-driver only if you "
                    "explicitly want the script to stop nmea_serial_driver."
                )
            print(f"Existing {args.topic} publisher detected; stopping nmea_serial_driver ...")
            stop_existing_nmea_driver()
            wait_for_no_publishers(deps, node, args.topic)

        print_header(args)
        if not args.yes:
            input("Place the vehicle at the fixed physical point, then press Enter to start ...")

        for round_id in range(1, args.rounds + 1):
            if round_id > 1:
                print(f"[round {round_id:02d}] restart wait {args.restart_wait_s:.1f}s ...")
                time.sleep(args.restart_wait_s)

            current_driver = start_gnss_driver(args, round_id)
            try:
                samples = collect_round(deps, node, current_driver, args, round_id)
                all_samples.extend(samples)
                print(f"[round {round_id:02d}] collected {len(samples)} valid samples")
            finally:
                stop_gnss_driver(current_driver)
                current_driver = None
                wait_for_no_publishers(deps, node, args.topic, timeout_s=3.0)

        raw_outputs: dict[str, Path] = {}
        round_means, summary = analyze_samples(deps, args, all_samples)
        if args.save_raw:
            raw_outputs = write_raw_outputs(args, all_samples, round_means, summary)
        output_path = plot_results(deps, args, all_samples, round_means, summary)
        if not args.save_driver_logs:
            shutil.rmtree(args.output_dir / ".driver_logs_tmp", ignore_errors=True)
        return output_path, summary, raw_outputs
    finally:
        stop_gnss_driver(current_driver)
        node.destroy_node()
        deps.rclpy.shutdown()


def main() -> int:
    args = parse_args()
    try:
        deps = preflight()
        output_path, summary, raw_outputs = run_experiment(deps, args)
    except KeyboardInterrupt:
        print("\nInterrupted. GNSS driver cleanup attempted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print("")
    print(f"Saved PNG: {output_path}")
    if raw_outputs:
        print(f"Saved samples CSV: {raw_outputs['samples']}")
        print(f"Saved round summary CSV: {raw_outputs['round_summary']}")
        print(f"Saved summary JSON: {raw_outputs['summary']}")
    print(
        "Summary: "
        f"max={summary['max_mean_distance_m']:.2f}m, "
        f"mean={summary['mean_mean_distance_m']:.2f}m, "
        f"rms={summary['rms_radius_m']:.2f}m, "
        f"samples={summary['total_valid_samples']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

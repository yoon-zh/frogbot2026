# XJTLU Autonomous Vehicle Documentation Index

> Last updated: 2026-10-31

## Core Documentation

| Topic | File |
|-------|------|
| System architecture and data flow | [architecture.md](architecture.md) |
| Common commands | [commands.md](commands.md) |
| Contributing guide (CN) | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |
| Contributing guide (EN) | [`../CONTRIBUTING-EN.md`](../CONTRIBUTING-EN.md) |
| Known issues and current blockers | [known_issues.md](known_issues.md) |
| Hardware specifications | [hardware_spec.md](hardware_spec.md) |
| FrogBoard mobile teleoperation & commands | [commands.md#frogboard-mobile-app-controlling](commands.md#frogboard-mobile-app-controlling) |
| WebRTC camera stream setup (go2rtc) | [`../scripts/camera_stream/README.md`](../scripts/camera_stream/README.md) |

## Technical Deep-Dive Documentation

| Topic | File |
|-------|------|
| FAST-LIO2 internals | [knowledge/fastlio2.md](knowledge/fastlio2.md) |
| PGO + GPS Factor | [knowledge/pgo.md](knowledge/pgo.md) |
| Nav2 tuning and runtime constraints | [knowledge/nav2_tuning.md](knowledge/nav2_tuning.md) |
| SLAM mapping and indoor navigation design plan | [knowledge/indoor_mapping_navigation_design.md](knowledge/indoor_mapping_navigation_design.md) |
| GPS global navigation and route planning | [knowledge/gps_planning.md](knowledge/gps_planning.md) |

## Development Log

- Found in `devlog/YYYY-MM.md`
- Latest updates: [October 2026 Devlog (Frogboard Teleoperation & WebRTC Streaming)](devlog/2026-10.md)

## Remote Teleoperation & Video Streaming

- **FrogBoard Server**: Headless WebSocket teleoperation server running over TLS/WSS with non-blocking PTY serial multiplexing ([`../scripts/frogboard_server/server.py`](../scripts/frogboard_server/server.py)).
- **Protocol & Safety Contract**: Downlink command formatting (`vcx`, `wc`, `en`), edge-triggered active braking, arming windows, and uplink telemetry synchronization ([`../scripts/frogboard_server/protocol.py`](../scripts/frogboard_server/protocol.py)).
- **WebRTC Camera Stream**: Ultra low-latency RGB video streaming for the Intel RealSense D455f via go2rtc ([`../scripts/camera_stream/README.md`](../scripts/camera_stream/README.md), [`../scripts/camera_stream/go2rtc.yaml`](../scripts/camera_stream/go2rtc.yaml)).
- **Service Daemons**: Background systemd units for autorun on boot ([`../scripts/frogboard_server/frogboard.service`](../scripts/frogboard_server/frogboard.service), [`../scripts/camera_stream/camera_stream.service`](../scripts/camera_stream/camera_stream.service)).

## Firmware Submodule

- The STM32 RM C Board lower-controller code is maintained as a git submodule: [`../src/firmware/frogbot-stm32/`](../src/firmware/frogbot-stm32/)
- Upstream repository: [`https://github.com/yoon-zh/frogbot-stm32`](https://github.com/yoon-zh/frogbot-stm32)
- To initialize and update the submodule:
  ```bash
  git submodule update --init --recursive
  ```

## Archive Notes

- `docs-CN/devlog/legacy/` and `docs-EN/devlog/legacy/` contain archived historical documents, kept for reference only and not maintained under the current structure.
- Documentation belonging to dependencies fetched through `dependencies.repos` is outside the scope of this repository's self-maintained documentation.

## Maintenance Principles

1. When code, launch files, parameters, system environment, or workflows change, update the corresponding documentation accordingly.
2. Monthly development records are appended to `devlog/YYYY-MM.md`.
3. New issues or status changes are synced to `known_issues.md`.
4. Command and workflow documentation must reflect the current executable state of the repository, not historical conventions.

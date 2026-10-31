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

## Firmware Snapshot

- The full STM32 RM C Board lower-controller directory is now checked into this repository: [`../src/firmware/rm_c_board/`](../src/firmware/rm_c_board/)
- This path is intentionally preserved as a repository-local snapshot, including the saved source tree, project files, and the companion build artifacts already stored with it
- Firmware-local readme: [`../src/firmware/rm_c_board/README.md`](../src/firmware/rm_c_board/README.md)

## Archive Notes

- `docs-CN/devlog/legacy/` and `docs-EN/devlog/legacy/` contain archived historical documents, kept for reference only and not maintained under the current structure.
- Documentation belonging to dependencies fetched through `dependencies.repos` is outside the scope of this repository's self-maintained documentation.

## Maintenance Principles

1. When code, launch files, parameters, system environment, or workflows change, update the corresponding documentation accordingly.
2. Monthly development records are appended to `devlog/YYYY-MM.md`.
3. New issues or status changes are synced to `known_issues.md`.
4. Command and workflow documentation must reflect the current executable state of the repository, not historical conventions.

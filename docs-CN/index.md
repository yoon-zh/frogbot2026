# XJTLU 自主导航车辆项目文档索引

> 最后更新: 2026-10-31

## 核心文档

| 主题 | 文件 |
|------|------|
| 系统结构与数据流 | [architecture.md](architecture.md) |
| 常用命令 | [commands.md](commands.md) |
| 贡献指南（中文） | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |
| 贡献指南（英文） | [`../CONTRIBUTING-EN.md`](../CONTRIBUTING-EN.md) |
| 已知问题与当前阻塞 | [known_issues.md](known_issues.md) |
| 硬件规格参数 | [hardware_spec.md](hardware_spec.md) |
| FrogBoard 手机端遥控与命令 | [commands.md#frogboard-mobile-app-controlling](commands.md#frogboard-mobile-app-controlling) |
| WebRTC 相机图传配置 (go2rtc) | [`../scripts/camera_stream/README.md`](../scripts/camera_stream/README.md) |

## 技术深度文档

| 主题 | 文件 |
|------|------|
| FAST-LIO2 工作原理 | [knowledge/fastlio2.md](knowledge/fastlio2.md) |
| PGO + GPS Factor | [knowledge/pgo.md](knowledge/pgo.md) |
| Nav2 调参与运行约束 | [knowledge/nav2_tuning.md](knowledge/nav2_tuning.md) |
| SLAM 建图与室内导航设计开发方案 | [knowledge/indoor_mapping_navigation_design.md](knowledge/indoor_mapping_navigation_design.md) |
| GPS 全局导航与路网规划 | [knowledge/gps_planning.md](knowledge/gps_planning.md) |

## 开发日志

- 在 `devlog/YYYY-MM.md`
- 最新记录：[2026年10月开发日志（Frogboard 遥控与 WebRTC 图传）](devlog/2026-10.md)

## 远程遥控与视频图传

- **FrogBoard Server**：基于 TLS/WSS 运行的无头模式 WebSocket 遥控服务器，具备非阻塞 PTY 串口复用能力（[`../scripts/frogboard_server/server.py`](../scripts/frogboard_server/server.py)）。
- **协议与安全保障**：下行控制指令格式化（`vcx`、`wc`、`en`）、边沿触发主动刹车、使能时间窗口与上行遥测同步（[`../scripts/frogboard_server/protocol.py`](../scripts/frogboard_server/protocol.py)）。
- **WebRTC 相机图传**：基于 go2rtc 实现的 Intel RealSense D455f RGB 超低延迟视频图传（[`../scripts/camera_stream/README.md`](../scripts/camera_stream/README.md)、[`../scripts/camera_stream/go2rtc.yaml`](../scripts/camera_stream/go2rtc.yaml)）。
- **后台服务守护进程**：用于开机自启的后台 systemd 服务单元（[`../scripts/frogboard_server/frogboard.service`](../scripts/frogboard_server/frogboard.service)、[`../scripts/camera_stream/camera_stream.service`](../scripts/camera_stream/camera_stream.service)）。

## 固件子模块

- STM32 RM C Board 下位机代码现作为 git submodule 维护：[`../src/firmware/frogbot-stm32/`](../src/firmware/frogbot-stm32/)
- 上游仓库：[`https://github.com/yoon-zh/frogbot-stm32`](https://github.com/yoon-zh/frogbot-stm32)
- 初始化并更新子模块：
  ```bash
  git submodule update --init --recursive
  ```

## 归档说明

- `docs-CN/devlog/legacy/` 与 `docs-EN/devlog/legacy/` 为历史原始文档归档，仅保留参考价值，不按当前结构继续维护。
- 通过 `dependencies.repos` 拉取的上游依赖文档不属于本项目自维护文档范围。

## 维护原则

1. 代码、launch、参数、系统环境、流程有变动时，同步更新对应文档。
2. 当月开发记录统一追加到 `devlog/YYYY-MM.md`。
3. 新问题或状态变化同步到 `known_issues.md`。
4. 命令和流程文档必须反映当前仓库真实可执行状态，而不是历史习惯。

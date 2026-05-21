# Robot Kinematic Viewer 👋
### 🤖 Interactive Kinematic Debug Viewer · IK · Playback · Safety Monitor

[English](#-english) | [中文](#-中文)

---

## 📑 Contents

- [🌍 English](#-english)
- [✨ Key Features](#-key-features)
- [🖼️ Preview](#️-preview)
- [💃 Dance Playback Demo](#-dance-playback-demo)
- [🚀 Quick Start](#-quick-start)
- [🎮 RViz IK Demo](#-rviz-ik-demo)
- [中文](#-中文)
- [✨ 主要能力](#-主要能力)
- [🖼️ 预览](#️-预览)
- [💃 跳舞回放演示](#-跳舞回放演示)
- [🚀 快速开始](#-快速开始)
- [🎮 RViz 联调示例](#-rviz-联调示例)

## 🌍 English

`robot_kinematic_viewer` is a practical desktop tool for robot kinematic debugging.
It provides a 3D interactive viewer with IK target manipulation, trajectory playback, and near-collision monitoring.

### ✨ Key Features

- Interactive 3D robot view (OpenGL + ImGui)
- Single-chain IK and full-body IK backend switching
- Keyframe recording and trajectory playback
- Safety panel with proxy-sphere distance monitoring
- User-defined obstacle editing and visualization
- Optional ROS bridge for external IK target input

### 🖼️ Preview

![Robot Kinematic Viewer — interactive 3D view, IK, and playback](assets/gui_viewer.jpg)

![Overview demo](assets/robot_kinematic_viewer.gif)

### 💃 Dance Playback Demo

Screen recording of Galbot G1 dance trajectory playback in the viewer (chassis slide-in + joint motion, imported from `/data/dance`):

**[assets/simplescreenrecorder-2026-05-21_11.21.13.mkv](assets/simplescreenrecorder-2026-05-21_11.21.13.mkv)** (~9 MB)

Default trajectory in the recording: `config/trajectories/galbot_g1_dance_slide_in.csv` (10 s). Other dance demos: `galbot_g1_dance_full.csv` (full ~56 s), `galbot_g1_dance_pure.csv`, `galbot_g1_dance_left_action1.csv` — see [Trajectory Playback](#-trajectory-playback-yaml--csv).

> MKV plays in most desktop players and browsers when opened locally; clone the repo or download the file from GitHub to watch.

### 🧠 Core Workflow

```text
Load URDF + config
  -> Interactive marker / UI target edits
  -> IK solve (single_chain or full_body)
  -> Scene update
  -> Collision monitor (proxy spheres)
  -> UI + line overlay feedback
```

### 🗂️ Repository Layout

```text
robot_kinematic_viewer/
  assets/                  # screenshots, GIFs, dance playback videos
  config/                  # YAML runtime configs + trajectories/
  scripts/                 # build/deploy, RViz, import_dance_trajectory.py
  include/                 # public headers
  src/                     # core implementation
  docs/                    # design docs
  deps/                    # vendored imgui/imguizmo/glad sources
```

### 🧰 Dependencies

- CMake >= 3.16
- C++17 compiler
- OpenGL, GLFW, GLEW, Assimp
- pinocchio, trac_ik, roscpp (ROS Noetic)
- yaml-cpp
- qpOASES (recommended for full-body backend)

### 🚀 Quick Start

Build only:

```bash
./build.sh
```

Full rebuild:

```bash
./all_rebuild.sh
```

Build + package runtime release bundle:

```bash
./auto_build.sh
```

Run with YAML config:

```bash
./bin/robot_kinematic_viewer config/robot_kinematic_viewer.yaml
```

Run directly with URDF path:

```bash
./bin/robot_kinematic_viewer /absolute/path/to/robot.urdf
```

### 🎮 RViz IK Demo

Start ROS + interactive marker + viewer stack:

```bash
./scripts/start_rviz_ik_stack.sh
```

Useful options:

```bash
./scripts/start_rviz_ik_stack.sh --ik-mode full_body --backend wbc_chain_ik --chain-index 0
./scripts/start_rviz_ik_stack.sh --pose-ik
```

### ⚙️ Configuration Notes

- Default startup config: `config/robot_kinematic_viewer.yaml`
- First CLI argument is treated as config path unless it ends with `.urdf`
- Update `robot.urdf_path` in YAML before first run on a new machine
- Keep top-level YAML keys limited to:
  - `window`, `robot`, `camera`, `ui`, `ik`, `ros`, `initial_pose`

### 🎬 Trajectory Playback (YAML / CSV)

See **[Dance Playback Demo](#-dance-playback-demo)** for a screen recording (`assets/simplescreenrecorder-2026-05-21_11.21.13.mkv`).

Playback panel supports `.yaml`, `.yml`, and `.csv`. **Preferred format** is a single table (same layout for YAML and CSV):

```text
joints: [optional chassis_x, chassis_y, chassis_yaw, joint1, joint2, ...]
values:
  - [time, optional chassis..., joint1, joint2, ...]
```

| Robot | Demo YAML | Demo CSV | Notes |
| --- | --- | --- | --- |
| Galbot G1 | `config/trajectories/galbot_g1_playback.yaml` | `config/trajectories/galbot_g1_playback.csv` | Includes 2D chassis track (`chassis_x/y/yaw`, rad) |
| Galbot G1 dance (arms) | — | `config/trajectories/galbot_g1_dance_left_action1.csv` | Arm segment, fixed chassis |
| Galbot G1 dance (chassis) | — | `config/trajectories/galbot_g1_dance_slide_in.csv` | 10s slide-in + spin (default) |
| Galbot G1 dance (chassis) | — | `config/trajectories/galbot_g1_dance_pure.csv` | ~4s segment with chassis |
| Galbot G1 dance (full) | — | `config/trajectories/galbot_g1_dance_full.csv` | ~56s full dance + chassis |
| Unitree G1 | `config/trajectories/unitree_g1_playback.yaml` | `config/trajectories/unitree_g1_playback.csv` | Joint-only (fixed base in viewer config) |

CSV header: `time,<joint columns...>` (chassis columns use the same names as YAML).

**Import more dances from `/data/dance`:**

```bash
python3 scripts/import_dance_trajectory.py \
  /data/dance/data/dance3/generate/pos/total_dance_data.csv \
  config/trajectories/galbot_g1_dance_full.csv --hz 30
```

The viewer also loads dance CSVs directly (`timestamp` column, `chassis_z` → yaw). Legacy compact YAML (`trajectory.dt` + separate `base_2d`) is still supported; **save** writes the table format above.

### 🛣️ Roadmap

- Replace visual-proxy collision with URDF collision geometry
- Improve pair-filter customization for collision checking
- Add more automated regression tests

---

## 中文

`robot_kinematic_viewer` 是一个面向机器人日常调试的桌面可视化工具。
它提供了 3D 交互视图、IK 目标操控、轨迹回放和近碰撞监控能力。

### ✨ 主要能力

- OpenGL + ImGui 交互式机器人 3D 视图
- `single_chain` / `full_body` IK 模式切换
- 关键帧录制与轨迹回放
- 基于代理球（proxy sphere）的最小距离安全监控
- 用户障碍物编辑与可视化
- 可选 ROS 外部目标位姿接入

### 🖼️ 预览

![Robot Kinematic Viewer — 交互界面](assets/gui_viewer.jpg)

![功能概览](assets/robot_kinematic_viewer.gif)

### 💃 跳舞回放演示

Galbot G1 在 Viewer 中回放跳舞轨迹的录屏（含底盘滑入与全身关节，轨迹来自 `/data/dance` 导入）：

**[assets/simplescreenrecorder-2026-05-21_11.21.13.mkv](assets/simplescreenrecorder-2026-05-21_11.21.13.mkv)**（约 9 MB）

录屏使用的默认轨迹：`config/trajectories/galbot_g1_dance_slide_in.csv`（10 s）。另有完整舞段 `galbot_g1_dance_full.csv`（约 56 s）、`galbot_g1_dance_pure.csv`、`galbot_g1_dance_left_action1.csv` 等，见 [轨迹回放](#-轨迹回放yaml--csv)。

> MKV 在本地用常见播放器或浏览器打开即可观看；需克隆仓库或从 GitHub 下载该文件。

### 🧠 核心流程

```text
加载 URDF 与配置
  -> UI/外部目标输入
  -> IK 求解
  -> 场景状态更新
  -> 距离监控与风险分级
  -> 侧边栏与3D连线反馈
```

### 🗂️ 仓库结构

```text
robot_kinematic_viewer/
  assets/                  # 截图、GIF、跳舞回放录屏
  config/                  # 运行配置 + trajectories/
  scripts/                 # 构建、RViz、import_dance_trajectory.py
  include/                 # 对外头文件
  src/                     # 核心实现
  docs/                    # 设计文档
  deps/                    # 内置三方源码（imgui/imguizmo/glad）
```

### 🧰 依赖环境

- CMake 3.16+
- C++17 编译器
- OpenGL / GLFW / GLEW / Assimp
- pinocchio / trac_ik / roscpp（ROS Noetic）
- yaml-cpp
- qpOASES（full-body 后端推荐）

### 🚀 快速开始

仅编译：

```bash
./build.sh
```

全量重编译：

```bash
./all_rebuild.sh
```

编译并生成可发布目录（含依赖收集）：

```bash
./auto_build.sh
```

使用配置文件启动：

```bash
./bin/robot_kinematic_viewer config/robot_kinematic_viewer.yaml
```

直接指定 URDF 启动：

```bash
./bin/robot_kinematic_viewer /绝对路径/robot.urdf
```

### 🎮 RViz 联调示例

一键拉起 `roscore + interactive marker + viewer`：

```bash
./scripts/start_rviz_ik_stack.sh
```

常见参数：

```bash
./scripts/start_rviz_ik_stack.sh --ik-mode full_body --backend wbc_chain_ik --chain-index 0
./scripts/start_rviz_ik_stack.sh --pose-ik
```

### ⚙️ 配置说明

- 默认入口配置：`config/robot_kinematic_viewer.yaml`
- 程序首个参数若以 `.urdf` 结尾则按 URDF 直接启动，否则按 YAML 配置加载
- 在新机器上请先修改 YAML 中 `robot.urdf_path`
- 顶层配置项需保持为：
  - `window`、`robot`、`camera`、`ui`、`ik`、`ros`、`initial_pose`

### 📚 文档

- 设计文档：`docs/ROBOT_KINEMATIC_VIEWER_DESIGN.md`

### 🎬 轨迹回放（YAML / CSV）

录屏演示见 **[跳舞回放演示](#-跳舞回放演示)**（`assets/simplescreenrecorder-2026-05-21_11.21.13.mkv`）。

侧边栏回放支持 `.yaml` / `.yml` / `.csv`。**推荐格式**为单表结构（YAML 与 CSV 列一致）：

```text
joints: [可选 chassis_x, chassis_y, chassis_yaw, 关节1, 关节2, ...]
values:
  - [时间, 可选底盘位姿..., 关节1, 关节2, ...]
```

| 机器人 | 示例 YAML | 示例 CSV | 说明 |
| --- | --- | --- | --- |
| Galbot G1 | `config/trajectories/galbot_g1_playback.yaml` | `config/trajectories/galbot_g1_playback.csv` | 含底盘平面轨迹（`chassis_x/y/yaw`，弧度） |
| Galbot G1 跳舞（手臂） | — | `config/trajectories/galbot_g1_dance_left_action1.csv` | 手臂段，底盘不动 |
| Galbot G1 跳舞（底盘） | — | `config/trajectories/galbot_g1_dance_slide_in.csv` | 10s 滑入+旋转（默认） |
| Galbot G1 跳舞（底盘） | — | `config/trajectories/galbot_g1_dance_pure.csv` | ~4s 含底盘片段 |
| Galbot G1 跳舞（完整） | — | `config/trajectories/galbot_g1_dance_full.csv` | ~56s 全程含底盘 |
| Unitree G1 | `config/trajectories/unitree_g1_playback.yaml` | `config/trajectories/unitree_g1_playback.csv` | 仅关节（配置中固定基座） |

CSV 表头：`time,<关节列...>`（底盘列名与 YAML 相同）。

**从 `/data/dance` 导入更多轨迹：**

```bash
python3 scripts/import_dance_trajectory.py \
  /data/dance/data/dance3/generate/pos/total_dance_data.csv \
  config/trajectories/galbot_g1_dance_full.csv --hz 30
```

Viewer 也可直接加载 dance 原始 CSV（`timestamp` 列、`chassis_z` 映射为 yaw）。旧版紧凑 YAML 仍可加载；**保存**时统一输出上表格式。

### 🛣️ 后续规划

- 用 URDF collision mesh 提升碰撞检测精度
- 完善碰撞对过滤策略可配置能力
- 补齐自动化回归测试

---

## 📄 License

TBD

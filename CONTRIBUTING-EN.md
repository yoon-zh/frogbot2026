> [!TIP]
> Chinese version: [CONTRIBUTING.md](CONTRIBUTING.md)

# Contributing Guide

## General Development Rules

> [!IMPORTANT]
> **Do not modify code directly on the vehicle (Jetson).** Any modifications found in the robot will be reset and erased.

1. Understand the code and architecture before modifying code or parameters
2. Do not modify the primary operating chain without verifying the impact scope
3. Upstream dependencies are fetched through `dependencies.repos` + `vcs import`
4. Critical modifications must explain what, why, and risk
5. Create your own branch from your computer, push changes to GitHub, then pull from the robot and test. The Jetson is only for compiling, running, and testing, do not modify code directly in there.


## Standard Development Flow

### Step 1: Session Initialization

SSH into the Jetson, check repository and hardware status:

```bash
ssh badger@robotfrogboard.yoonzh.com
git pull
```

First-time deployment or new machine initialization:

```bash
make setup
make build
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/init_runtime_data.sh
```

Manually check hardware status:

```bash
# Check LiDAR (Livox MID360)
ls /dev/ttyUSB* /dev/ttyACM*
ros2 topic echo /livox/lidar --once

# Check GPS
ros2 topic echo /fix --once

# Check IMU (BMI088, integrated in RM C Board)
ros2 topic echo /imu --once

# Check STM32 serial port
ls /dev/stm32_board
```

Confirm all hardware is functioning before starting work.


### Step 2: Receive Task and Create/Switch Branch

After receiving a task, create a new branch from the latest `main`, or switch to an existing working branch:

```bash
# New task: create new branch
git checkout -b <BRANCH-NAME>

# Continue existing task: switch to existing branch and sync
git checkout <BRANCH-NAME>
git pull origin <BRANCH-NAME>
```

Branch names must be descriptive (changing what, doing what):

- Good Examples: `rtk-signal`, `nav-tuning`, `lidar-fix`, `docs-sync`
- Bad Examples: `gps`, `lio`, `frc`

Use one branch per task. Do not mix unrelated work in a single branch.


### Step 3: Research and Preparation

Before writing any code:

1. Define the problem: Write specifically what bug or feature you will work on
2. Check relevant files to your task
3. Read launch chain from the `Makefile` & launch file
4. Read parameters in `master_params.yaml` and related YAML configs, confirm current parameter values.
5. Decide what needs to change and why.
6. Ask questions. If you cannot find something in the documentation, feel free to discuss with us.

### Step 4: Code and commit to Git

When you feel comfortable, you can start adding small changes and testing them in the robot. For any bigger changes, make sure to talk with us first.

First, upload your changes from your computer:
```bash
git add .
git commit -m "<COMMIT_MSG>"
git push -u origin <BRANCH_NAME>
```

For commit messages, use a common convention such as: https://gist.github.com/qoomon/5dfcdf8eec66a051ecd85625518cfd13


### Step 5: Test in the robot

SSH into the robot with `ssh badger@robotfrogboard.yoonzh.com`. Then:

```bash
git fetch
git switch <BRANCH_NAME>
git pull
```

If you changed any code under `src/`, you must build again. Configs do not apply. Common build commands:
```bash
# Layered build
mbuild build-bringup
mbuild build-fastlio2
mbuild build-sensor
mbuild build-perception
mbuild build-planning
mbuild build-navigation

# Single package build
colcon build --packages-select <pkg> --symlink-install --parallel-workers 1
ss
```
Using `mbuild build-*` automatically sources the new workspace. If you use `colcon build` manually, remember to keep `--parallel-workers 1` and source after building.

If successful, launch the system in the appropriate mode based on the scope of changes:
```bash
make launch-slam
make launch-explore
make launch-indoor-nav
make launch-corridor
make launch-travel
make launch-explore-gps
make launch-nav-gps
make launch-rtk-basic
make launch-survey
make launch-tightly-coupled
```

At minimum, verify the following:
- All relevant nodes are online
- Key topics / actions have data
- `map -> odom -> base_footprint -> base_link` TF chain is complete (no missing links in Foxglove)
- When moving the robot, see if any unexpected behavior occurs
- Session logs appear in `runtime-data/logs/latest/`

### Step 6: Verify your changes

Before opening a PR, check:

1. Do all launch modes compile successfully on the robot?
2. Does it change the existing launch chain?
3. Which launch modes, nodes, or topics are affected by the new changes?
4. Are new dependencies needed? If so, are `dependencies.repos`,  `package.xml` and `CMakeLists.txt` all updated?
5. Are parameter changes correctly reflected in `master_params.yaml` or other relevant config?
6. Does the changes introduce any risk points?

> [!NOTE]
> Make sure you test ALL the launch modes. If any shutdown or crash occurs, save the logs and see what happened.
>
> For other parts, you may use an agentic AI to help check these considerations. If there is any code you don't fully understand, ask us.


### Step 7: Documentation

By this point, we assume you have completed your task and it works properly in the robot. Then the last step is to document it. At minimum, you must update the following:
- Any new or affected commands should be updated in [commands.md](docs-EN/commands.md)
- A summary of all changes must be documented in the newest devlog
- You MUST update both the Chinese and English version. You may write it in your native language, then ask AI to translate it to the other.

> If you changed any parameter in [master_params.yaml](src/bringup/config/master_params.yaml), you MUST write a thorough analysis of why, how it impacts the robot, and which edge cases should be considered.

The devlog must follow this format:
```md
## YYYY.MM.DD

### [Change Topic Title]

#### File
- `file1/`
- `file2/`

#### Change
#### Reason
#### Effect
```

After you are done, make sure you have uploaded all your changes with `git push`.

### Step 8: Pull request

After uploading everything, go to GitHub and create a pull request. Follow the default format and checklist.
- Most recent repo: https://github.com/yoon-zh/frogbot2026
- Original repo this fork comes from: https://github.com/Yangbadger222/XJTLU-autonomous-vehicle-rtk
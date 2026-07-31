# Real-Exp 全流程指南（zh-Hans）

本文覆盖当前双臂 FR3 项目的完整流程：仓库与环境配置、ROS 编译、GELLO 遥操作数采、可选的 GELLO 主动回正与通电保持、数据集校验和传输、ACT 与 Diffusion 训练、Weights & Biases 记录、远程 Policy Server、Robot Host 端 dry-run、启动姿态对齐、限速执行、停机和故障排查。

本文以当前源码为准，包含下列新增入口和参数：

- `scripts/start_collection_duo.sh`
- `scripts/start_deployment_duo.sh`
- `data_collection/lerobot_collection.py --gello-reset-hold`
- `train/franka_act_policy_executor.py --policy-start | --episode-start [INDEX] --limit`
- `train/franka_diffusion_policy_executor.py --policy-start | --episode-start [INDEX] --limit`

## 1. 占位符规则

所有由用户、任务、机器或实验决定的值都写成 `<含义明确的占位符>`。

复制命令前必须替换全部尖括号内容。不能把尚未替换的 `<...>` 原样交给 shell，否则 `<` 和 `>` 可能被解释成输入、输出重定向。

常用占位符：

| 占位符 | 应传入的内容 |
| --- | --- |
| `<ROBOT_REPO_ROOT>` | Robot Host 上本仓库的绝对路径 |
| `<TRAIN_REPO_ROOT>` | 训练主机上本仓库的绝对路径 |
| `<POLICY_REPO_ROOT>` | Policy Server 上本仓库的绝对路径 |
| `<ROBOT_LEROBOT_ENV>` | Robot Host 上包含 LeRobot 和执行器依赖的 Conda 环境名 |
| `<TRAIN_LEROBOT_ENV>` | 训练主机上的 LeRobot/PyTorch Conda 环境名 |
| `<POLICY_LEROBOT_ENV>` | Policy Server 上的 LeRobot/PyTorch Conda 环境名 |
| `<CONDA_ACTIVATION_SCRIPT>` | 当前机器 Conda 的 `bin/activate` 路径 |
| `<FRANKA_WS_SETUP>` | Franka ROS 2 工作区的 `install/setup.bash` 路径 |
| `<DATASET_REPO_ID>` | 写入 LeRobot 元数据的 `命名空间/数据集名` |
| `<ROBOT_DATASET_ROOT>` | Robot Host 上的数据集根目录 |
| `<TRAIN_DATASET_ROOT>` | 训练主机上的同一数据集副本 |
| `<TASK_DESCRIPTION>` | 数采和推理使用的自然语言任务描述 |
| `<ACT_OUTPUT_DIR>` | ACT 独立训练输出目录 |
| `<DIFFUSION_OUTPUT_DIR>` | Diffusion 独立训练输出目录 |
| `<POLICY_PATH_ON_SERVER>` | Policy Server 可访问的 `.../checkpoints/last/pretrained_model` 路径 |
| `<POLICY_SERVER_IP>` | Robot Host 能访问的 Policy Server IP |
| `<POLICY_SERVER_PORT>` | Policy Server 的 gRPC 端口 |
| `<ROBOT_SSH_USER>`、`<ROBOT_SSH_HOST>`、`<ROBOT_SSH_PORT>` | Robot Host 的 SSH 用户名、地址和端口 |
| `<TRAIN_SSH_USER>`、`<TRAIN_SSH_HOST>`、`<TRAIN_SSH_PORT>` | 训练主机的 SSH 用户名、地址和端口 |
| `<POLICY_SSH_USER>`、`<POLICY_SSH_HOST>`、`<POLICY_SSH_PORT>` | Policy Server 的 SSH 用户名、地址和端口 |
| `<FPS>` | 数采、数据集元数据、Policy Server 和执行器统一使用的频率 |

## 2. 机器分工与进程边界

### Robot Host

Robot Host 与两台 Franka、两套 GELLO、两套夹爪和所有相机物理相连，负责：

- ROS 2 硬件接口和状态发布
- 数采控制器或部署控制器
- 夹爪客户端
- 相机发布器
- ROS 与 ZMQ 之间的 Bridge
- 数采时的 LeRobot Recorder
- 部署时的 ACT 或 Diffusion Executor

### 训练主机

训练主机保存数据集副本，训练 ACT 和 Diffusion。两个训练任务可以分别放在不同的 `tmux` 会话，并使用不同 GPU。

### Policy Server

Policy Server 接收 Robot Host 发来的 observation，在 GPU 上完成推理并返回 action chunk。Executor 的 `--policy-path` 因此必须是 Policy Server 能访问的路径，而不是只在 Robot Host 存在的路径。

训练主机和 Policy Server 可以是同一台物理机器，但终端和职责仍应分开。

### 当前进程图和新增变化

| 进程或 Supervisor | 主机 | 当前行为/变化 |
| --- | --- | --- |
| `start_collection_duo.sh` | Robot | 一次启动并监管五组数采 ROS 进程；Recorder 有意保持独立 |
| GELLO Publisher | Robot | 只用于遥操作数采；参数服务同时支持 Recorder 管理的主动回正/保持 |
| Collection Arm Bringup | Robot | 使用普通遥操作 Controller 路径，不进入 Deployment Mode |
| Collection Bridge | Robot | 使用 `example_duo.yaml`，发布同步录制数据包 |
| `lerobot_collection.py` | Robot | 新增 `--gello-reset-hold` 和 `r`；不加参数时旧行为完全不变 |
| `start_deployment_duo.sh` | Robot | Preflight、启动、就绪检查、持续监管并安全反序关闭四组部署进程 |
| Deployment Arm Bringup | Robot | 使用 `deployment_mode:=true`；Controller 初始 inactive，Bridge 激活且收到真实命令后才工作 |
| Gripper Client | Robot | 两个一键流程都先左后右启动，避免并发初始化超时 |
| Deployment Bridge | Robot | 使用 `deployment_duo.yaml`，以 standby 启动，发布 observation、接收 command 并控制 Controller Gate |
| Policy Server | GPU 主机 | 加载 Executor 请求的服务器侧 checkpoint；支持固定或随机 Diffusion Noise |
| ACT/Diffusion Executor | Robot | 自动激活/关闭 Bridge，支持启动对齐、可选关节动作拉长、Action Fusion、有限 Last-Command Hold 和部署日志 |

### SSH 连接模板

这些命令不依赖 `~/.ssh/config`：

```bash
ssh -p <ROBOT_SSH_PORT> <ROBOT_SSH_USER>@<ROBOT_SSH_HOST>
```

```bash
ssh -p <TRAIN_SSH_PORT> <TRAIN_SSH_USER>@<TRAIN_SSH_HOST>
```

```bash
ssh -p <POLICY_SSH_PORT> <POLICY_SSH_USER>@<POLICY_SSH_HOST>
```

训练和 Policy Serving 使用同一台机器时，也要分别打开 SSH 终端或 `tmux` 会话。

## 3. 安全规则

1. 清空工作空间后再解锁 Franka 并开启 FCI。
2. 第一次实机执行时必须有人守在急停旁边。
3. 数采控制器栈与部署控制器栈不能同时运行。
4. Policy 部署期间不要启动 GELLO Publisher。
5. ROS Arm Controller 占用机械臂时，不要运行直接控制 Franka 的 `pylibfranka` 初始化工具。
6. 先运行不带 `--execute` 的 dry-run，确认维度、相机键、延迟和 Policy 输出有限且稳定。
7. `--limit` 只检查和拉长关节空间动作，不等于完成笛卡尔碰撞或任务安全验证。
8. 出现异常运动立即停止，不要在未确认触发命令前反复清除 Franka reflex。

## 4. 仓库与环境配置

### 4.1 Clone 并初始化子模块

在需要源码的每台机器上执行：

```bash
git clone --recurse-submodules <REAL_EXP_GIT_URL> <REPO_DESTINATION>
cd <REPO_DESTINATION>
git submodule update --init --recursive
```

### 4.2 编译 Robot Host 的 ROS Overlay

这里使用系统 ROS Python，不要使用 LeRobot Conda 环境：

```bash
cd <ROBOT_REPO_ROOT>/gello_software/ros2
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
colcon build --symlink-install
source install/setup.bash
```

修改 `gello_software/ros2/` 后需要重新编译。每个新 ROS 终端都要重新 source overlay。

### 4.3 创建 LeRobot 环境

在需要录制、训练或推理的机器上分别执行：

```bash
source <CONDA_ACTIVATION_SCRIPT>
conda create -n <LEROBOT_ENV_NAME> python=<PYTHON_VERSION> -y
conda activate <LEROBOT_ENV_NAME>

<INSTALL_COMMAND_FOR_PYTORCH_MATCHING_THIS_HOST_AND_CUDA>
python -m pip install uv

cd <REPO_ROOT_ON_THIS_HOST>/lerobot
uv pip install -e .
cd <REPO_ROOT_ON_THIS_HOST>
```

Editable install 可能替换先前安装的 PyTorch，安装结束后必须再次验证最终环境：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'no cuda')"
python -c "import lerobot, wandb, zmq, pyarrow; print('Python environment OK')"
```

如果 TorchCodec 报 `libavutil.so` 缺失，在当前环境安装兼容 FFmpeg 并导出动态库路径：

```bash
conda install -c conda-forge "ffmpeg=<COMPATIBLE_FFMPEG_MAJOR>.*" -y
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

### 4.4 Robot Host 的两类终端环境

ROS Stack 终端：

```bash
cd <ROBOT_REPO_ROOT>
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
source <ROBOT_REPO_ROOT>/gello_software/ros2/install/setup.bash
```

Recorder 或 Executor 终端：

```bash
cd <ROBOT_REPO_ROOT>
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
source <ROBOT_REPO_ROOT>/gello_software/ros2/install/setup.bash
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

Executor 会通过 `ros2` CLI 调用 `/set_deployment_active`，因此 Executor 终端也必须有 ROS 环境。

## 5. 数据采集

### 5.1 数据集实际保存什么

当前双臂 Bridge 和 Recorder 保存：

- `observation.state`：左臂、左夹爪、右臂、右夹爪的实测状态
- `action`：下一采样时刻的绝对关节目标和夹爪命令
- `observation.images.cam_left`
- `observation.images.cam_front`
- `observation.images.cam_right`
- `meta/real_exp_action_config.json`：动作表示和维度契约

双臂常见布局为 `[左臂7维, 左夹爪1维, 右臂7维, 右夹爪1维]`。训练前仍必须读取实际元数据，不能仅凭经验假定布局。

### 5.2 数采前硬件检查

1. 核对 `gello_duo.yaml` 使用的左右 GELLO USB 设备路径。
2. GELLO 重连或机构变化后核对左右关节 offset。
3. 核对 `example_fr3_duo_config.yaml` 中的两台 Franka IP 和 namespace。
4. 在 `example_three_cameras.yaml` 固定相机序列号与名称的对应关系。
5. 解锁 Franka、开启 FCI、清空双臂工作空间并给夹爪上电。
6. 确认 observation ZMQ 端口没有被旧 Bridge 占用。

Publisher 启动后检查 GELLO 和 Franka 状态：

```bash
ros2 topic echo --once /left/gello/joint_states
ros2 topic echo --once /right/gello/joint_states
ros2 topic echo --once /left/franka/joint_states
ros2 topic echo --once /right/franka/joint_states
```

### 5.3 一键启动数采 ROS Stack

Robot Host 终端 1，使用系统 ROS 环境：

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_collection_duo.sh
```

一键脚本的可选覆盖项使用环境变量，而不是命令行参数：

```bash
READY_TIMEOUT=<STARTUP_TIMEOUT_SECONDS> \
COLLECTION_LOG_DIR=<COLLECTION_STACK_LOG_DIR> \
COLLECTION_ZMQ_PORT=<COLLECTION_OBSERVATION_PORT> \
bash scripts/start_collection_duo.sh
```

如果修改数采端口，必须同步修改 `example_duo.yaml` 并重新编译 ROS Overlay，确保 Bridge 监听同一端口。

脚本按依赖顺序启动五组进程：

| 顺序 | 进程组 | 当前命令/配置 | 作用 |
| --- | --- | --- | --- |
| 1 | GELLO | `franka_gello_state_publisher` + `gello_duo.yaml` | 发布左右遥操作器关节目标 |
| 2 | Arms | `franka_fr3_arm_controllers` + `example_fr3_duo_config.yaml` | 普通遥操作控制路径，不启用 deployment mode |
| 3 | Grippers | 先左后右依次启动 | 避免双夹爪并发初始化造成 action timeout |
| 4 | Cameras | `example_three_cameras.yaml` | 发布三个命名 RGB 流 |
| 5 | Bridge | `example_duo.yaml` | 通过 ZMQ 发布同步数采数据包 |

脚本会检查端口、等待真正收到 topic 数据、持续监控所有子进程、把日志写到 `log/collection/` 下以时间戳命名的目录，并在 Ctrl-C 时按反序清理完整进程树。

它有意不启动 `lerobot_collection.py`。Recorder 独立放在第二个 Conda 终端，以便持续看到录制按键，并防止 Recorder 异常直接拆掉硬件 Stack。

### 5.4 启动 Recorder

Robot Host 终端 2，使用 LeRobot Conda 环境：

```bash
cd <ROBOT_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python data_collection/lerobot_collection.py \
  --host <COLLECTION_BRIDGE_HOST> \
  --port <COLLECTION_OBSERVATION_PORT> \
  --repo-id <DATASET_REPO_ID> \
  --local-dir <ROBOT_DATASET_ROOT> \
  --fps <FPS> \
  --task "<TASK_DESCRIPTION>"
```

Recorder 按键：

| 输入 | 功能 |
| --- | --- |
| `s` + Enter | 开始一个新 episode |
| `e` + Enter | 结束并保存当前 episode |
| `d` + Enter | 丢弃当前 episode 缓冲区 |
| `q` + Enter | 退出、完成元数据/统计量写入；若仍有未保存帧则按现有退出逻辑保存 |

Recorder 会续写契约一致的已有数据集。如果 live stream 与已有数据集的 feature/action 契约不同，它会创建新的同级目录，而不是把不兼容数据静默追加进去。

### 5.5 可选 GELLO 主动回正与保持

需要该功能时才添加 `--gello-reset-hold`：

```bash
python data_collection/lerobot_collection.py \
  --host <COLLECTION_BRIDGE_HOST> \
  --port <COLLECTION_OBSERVATION_PORT> \
  --repo-id <DATASET_REPO_ID> \
  --local-dir <ROBOT_DATASET_ROOT> \
  --fps <FPS> \
  --task "<TASK_DESCRIPTION>" \
  --gello-reset-hold
```

此参数新增 `r` + Enter：

1. `r` 将已知 Franka 初态通过映射公式逆向转换为两侧 GELLO 原始关节目标。
2. 两侧 GELLO 主动移动到目标并持续通电保持。
3. `s` 开始录制，同时启用“检测到真实运动后释放”。
4. 操作者实际移动任一 GELLO 关节后才关闭扭矩，避免启动瞬间目标跳变。
5. `q`、Ctrl-C 或 Recorder 清理流程都会关闭 GELLO 扭矩。

Helper 由 Recorder 自动管理，并复用正在运行的 GELLO Publisher 参数服务。不要同时运行其它主动 GELLO 驱动程序。Episode 正在录制时，`r` 会被拒绝。

### 5.6 手动启动数采进程，仅用于诊断

正常流程优先使用一键脚本。需要隔离某个组件时，仍必须保持相同顺序：

```bash
ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=gello_duo.yaml
```

```bash
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=example_fr3_duo_config.yaml
```

```bash
ros2 run franka_gripper_manager franka_gripper_client \
  --ros-args -r __node:=franka_gripper_client -r __ns:=/left
```

左夹爪初始化完成后再启动右夹爪：

```bash
ros2 run franka_gripper_manager franka_gripper_client \
  --ros-args -r __node:=franka_gripper_client -r __ns:=/right
```

```bash
ros2 launch franka_realsense_camera_publisher cameras.launch.py \
  config_file:=example_three_cameras.yaml
```

```bash
ros2 launch franka_lerobot_data_bridge bridge.launch.py \
  config_file:=example_duo.yaml
```

数采时不要添加 `deployment_mode:=true`，也不要使用 `deployment_duo.yaml`。

### 5.7 正确结束数采

1. 在 Recorder 中保存或丢弃正在录制的 episode。
2. 输入 `q` + Enter，等待数据集 finalize 完成。
3. 在 `start_collection_duo.sh` 终端按 Ctrl-C。
4. 启动其它 Stack 前确认 ZMQ 端口和 Franka 连接均已释放。

## 6. 数据集校验、编辑和传输

### 6.1 数采结束后立即校验

```bash
cd <ROBOT_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>

python data_collection/validate_dataset.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --verbose
```

Validator 会检查 episode/frame 连续性、元数据总量、state/action 维度、时间戳、视频、夹爪范围和可疑关节动作跳变。警告意味着需要调查，不代表应该自动对整条轨迹做平滑。

如果当前环境没有 OpenCV，只跳过实际 MP4 帧数检查：

```bash
python data_collection/validate_dataset.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --skip-video-frames
```

### 6.2 删除坏 episode，不要手工改 Parquet

先 dry-run：

```bash
python data_collection/delete_lerobot_episode.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --episode-indices <SPACE_SEPARATED_EPISODE_INDICES> \
  --dry-run
```

输出到新数据集目录：

```bash
python data_collection/delete_lerobot_episode.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --episode-indices <SPACE_SEPARATED_EPISODE_INDICES> \
  --output-dir <EDITED_DATASET_ROOT>
```

完成后再次校验。在处理后副本通过校验之前，保留原始数据集不变。

### 6.3 传到训练主机

Robot Host 能直接访问训练主机时：

```bash
rsync -avhP \
  -e "ssh -p <TRAIN_SSH_PORT>" \
  <ROBOT_DATASET_ROOT>/ \
  <TRAIN_SSH_USER>@<TRAIN_SSH_HOST>:<TRAIN_DATASET_ROOT>/
```

没有直连路由时，通过本地工作站分两步传输。必须保留完整数据集根目录，包括 `data/`、`videos/` 和 `meta/`。

在训练主机再次校验：

```bash
cd <TRAIN_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>

python data_collection/validate_dataset.py \
  --dataset-root <TRAIN_DATASET_ROOT>
```

## 7. 使用 W&B 训练 ACT 和 Diffusion

### 7.1 每个训练账号只需登录一次 W&B

```bash
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>
wandb login
```

需要在线记录时不要添加 `--disable-wandb`。

### 7.2 在 `tmux` 中训练 ACT

```bash
tmux new -s <ACT_TRAIN_TMUX_SESSION>
```

进入会话后：

```bash
cd <TRAIN_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

CUDA_VISIBLE_DEVICES=<ACT_TRAIN_GPU_ID> \
python train/train_lerobot_policy.py \
  --dataset-root <TRAIN_DATASET_ROOT> \
  --dataset-repo-id <DATASET_REPO_ID> \
  --policy-type act \
  --output-dir <ACT_OUTPUT_DIR> \
  --steps <ACT_TOTAL_TRAINING_STEPS> \
  --batch-size <ACT_BATCH_SIZE> \
  --num-workers <DATALOADER_WORKERS> \
  --save-freq <CHECKPOINT_INTERVAL_STEPS> \
  --log-freq <LOG_INTERVAL_STEPS> \
  --val-ratio <VALIDATION_EPISODE_RATIO> \
  --val-freq <VALIDATION_INTERVAL_STEPS> \
  --wandb-project <WANDB_PROJECT_NAME> \
  --act-chunk-size <ACT_TRAIN_CHUNK_SIZE> \
  --act-kl-weight <ACT_KL_WEIGHT> \
  --seed <TRAINING_RANDOM_SEED>
```

使用 Ctrl-B，再按 D 离开会话。重新连接：

```bash
tmux attach -t <ACT_TRAIN_TMUX_SESSION>
```

### 7.3 在第二个 `tmux` 会话训练 Diffusion

```bash
tmux new -s <DIFFUSION_TRAIN_TMUX_SESSION>
```

进入会话后：

```bash
cd <TRAIN_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

CUDA_VISIBLE_DEVICES=<DIFFUSION_TRAIN_GPU_ID> \
python train/train_lerobot_policy.py \
  --dataset-root <TRAIN_DATASET_ROOT> \
  --dataset-repo-id <DATASET_REPO_ID> \
  --policy-type diffusion \
  --output-dir <DIFFUSION_OUTPUT_DIR> \
  --steps <DIFFUSION_TOTAL_TRAINING_STEPS> \
  --batch-size <DIFFUSION_BATCH_SIZE> \
  --num-workers <DATALOADER_WORKERS> \
  --save-freq <CHECKPOINT_INTERVAL_STEPS> \
  --log-freq <LOG_INTERVAL_STEPS> \
  --val-ratio <VALIDATION_EPISODE_RATIO> \
  --val-freq <VALIDATION_INTERVAL_STEPS> \
  --wandb-project <WANDB_PROJECT_NAME> \
  --diffusion-horizon <DIFFUSION_HORIZON> \
  --diffusion-n-obs-steps <DIFFUSION_OBSERVATION_STEPS> \
  --diffusion-noise-scheduler-type <DDPM_OR_DDIM> \
  --diffusion-num-inference-steps <DIFFUSION_INFERENCE_STEPS> \
  --seed <TRAINING_RANDOM_SEED>
```

ACT 和 Diffusion 并行时使用不同 GPU ID。`CUDA_VISIBLE_DEVICES` 会把当前进程看到的那张物理卡重新映射成进程内的 `cuda:0`。

### 7.4 续训

数据集、Policy 类型、Policy 结构参数和输出目录必须与原训练一致。添加 `--resume`，并把 `--steps` 设为新的最终总步数：

```bash
CUDA_VISIBLE_DEVICES=<TRAIN_GPU_ID> \
python train/train_lerobot_policy.py \
  --dataset-root <TRAIN_DATASET_ROOT> \
  --dataset-repo-id <DATASET_REPO_ID> \
  --policy-type <ACT_OR_DIFFUSION> \
  --output-dir <EXISTING_OUTPUT_DIR> \
  --steps <NEW_FINAL_TOTAL_STEPS> \
  <REPEAT_THE_ORIGINAL_POLICY_AND_VALIDATION_ARGUMENTS> \
  --wandb-project <WANDB_PROJECT_NAME> \
  --resume
```

不要对空目录或其它实验目录使用 `--resume`。不带 `--resume` 时，Trainer 会主动拒绝覆盖已有输出目录。

### 7.5 确认训练产物

可部署 checkpoint 位于：

```text
<TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model
```

部署前检查：

```bash
test -f <TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model/config.json
ls -lh <TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model
```

### 7.6 Policy Server 与训练主机不同时复制产物

复制完整训练输出，避免 checkpoint 链接、Processor 文件和模型文件分离：

```bash
rsync -avhP \
  -e "ssh -p <POLICY_SSH_PORT>" \
  <TRAIN_OUTPUT_DIR>/ \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST>:<POLICY_OUTPUT_DIR>/
```

至少复制 `inspect` 所需的数据集元数据：

```bash
ssh -p <POLICY_SSH_PORT> \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST> \
  "mkdir -p <POLICY_SERVER_DATASET_ROOT>/meta"

rsync -avhP \
  -e "ssh -p <POLICY_SSH_PORT>" \
  <TRAIN_DATASET_ROOT>/meta/ \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST>:<POLICY_SERVER_DATASET_ROOT>/meta/
```

传输后，该机器上的 `<POLICY_PATH_ON_SERVER>` 应解析为 `<POLICY_OUTPUT_DIR>/checkpoints/last/pretrained_model`。

## 8. 部署训练好的 Policy

部署包含三个相互独立的层次：

1. Robot Host ROS Deployment Stack：四组硬件进程。
2. Policy Server：GPU 主机上的推理进程。
3. Robot Host Executor：把前两层连接起来的 ACT 或 Diffusion 进程。

部署不使用 GELLO。

### 8.1 数采和部署配置的关键区别

| 项目 | 数据采集 | Policy 部署 |
| --- | --- | --- |
| GELLO Publisher | 必须 | 禁止 |
| Arm Launch | 普通遥操作模式 | `deployment_mode:=true` |
| Deployment Joint Controller | 不使用 | 加载为 inactive，仅在 Bridge 收到命令后激活 |
| Camera YAML | `example_three_cameras.yaml` | `example_three_cameras.yaml` |
| Bridge YAML | `example_duo.yaml` | `deployment_duo.yaml` |
| ZMQ Observation | 数采同步包 | Policy 实时 observation |
| ZMQ Command | 不使用 | Policy action 返回 Bridge |
| Bridge 初始状态 | 持续数采流 | `STANDBY` |

### 8.2 不连接 Franka 的预检查

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_deployment_duo.sh --preflight-only
```

Preflight 会验证：

- ROS setup 文件和必要系统命令
- 已安装 ROS 包确实来自当前 checkout 的 overlay
- Arm、Gripper、Camera 和 Bridge YAML 契约
- Deployment Bridge 模式、state source、topic、service 和端口
- 没有旧 GELLO、Bridge 或 Controller Stack 冲突
- Observation 和 Command 端口空闲

所有 preflight 错误都必须先修复，不能带错启动硬件。

### 8.3 一键启动部署所需四组 ROS 进程

Robot Host 终端 1，使用系统 ROS 环境：

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_deployment_duo.sh
```

脚本依次启动：

1. Franka 硬件、Robot/Joint State Publisher，以及 `example_fr3_duo_config.yaml` + `deployment_mode:=true` 加载的 inactive Deployment Controller。
2. 先左后右启动两侧 Gripper Client。
3. 使用 `example_three_cameras.yaml` 启动三个 Camera Publisher。
4. 使用 `deployment_duo.yaml` 启动 Bridge，包括 Observation ZMQ、Command ZMQ 和 `/set_deployment_active`。

它不会启动 Policy Server 或 Executor。只有看到下列输出才能继续：

```text
All four deployment ROS groups are ready in STANDBY mode
```

日志在 `log/deployment_stack/` 下以时间戳命名的目录。脚本会定期检查运行端点，连续三次失败会停止整套受监管进程。

需要覆盖默认值时，在命令前设置环境变量：

```bash
READY_TIMEOUT=<STARTUP_TIMEOUT_SECONDS> \
DEPLOYMENT_LOG_DIR=<DEPLOYMENT_STACK_LOG_DIR> \
DEPLOYMENT_OBSERVATION_PORT=<OBSERVATION_ZMQ_PORT> \
DEPLOYMENT_COMMAND_PORT=<COMMAND_ZMQ_PORT> \
bash scripts/start_deployment_duo.sh
```

如果覆盖端口，已安装的 `deployment_duo.yaml` 也必须使用相同值，否则 preflight 会拒绝启动。

### 8.4 在 Policy Server 检查 checkpoint

```bash
cd <POLICY_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <POLICY_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python train/deploy_lerobot_policy.py inspect \
  --policy-path <POLICY_PATH_ON_SERVER> \
  --dataset-root <POLICY_SERVER_DATASET_ROOT>
```

核对 Policy 类型、最大 action chunk、FPS、state/action 维度、相机键和动作表示。Inspect 需要数据集元数据副本，但真正推理仍从 checkpoint 加载模型。

### 8.5 启动 Policy Server

Policy Server 终端 2：

```bash
cd <POLICY_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <POLICY_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

CUDA_VISIBLE_DEVICES=<POLICY_SERVER_GPU_ID> \
python train/deploy_lerobot_policy.py server \
  --host <POLICY_SERVER_BIND_ADDRESS> \
  --port <POLICY_SERVER_PORT> \
  --fps <FPS> \
  --inference-latency <TARGET_INFERENCE_LATENCY_SECONDS> \
  --obs-queue-timeout <OBSERVATION_QUEUE_TIMEOUT_SECONDS> \
  --diffusion-noise-scheduler-type <DDPM_OR_DDIM> \
  --diffusion-num-inference-steps <DIFFUSION_INFERENCE_STEPS> \
  --diffusion-fixed-noise-seed <DIFFUSION_NOISE_SEED>
```

Server 启动命令本身不传 checkpoint。Executor 连接后会发送 `<POLICY_PATH_ON_SERVER>`、Policy 类型、chunk 长度和 device，因此同一 Server 会为当前 Executor 加载其请求的 ACT 或 Diffusion checkpoint。

需要随机 Diffusion 噪声时，用 `--disable-diffusion-fixed-noise` 替代固定 seed 参数。

从 Robot Host 检查网络：

```bash
nc -vz <POLICY_SERVER_IP> <POLICY_SERVER_PORT>
```

### 8.6 ACT dry-run

Robot Host 终端 3，使用 ROS + LeRobot Conda 环境：

```bash
cd <ROBOT_REPO_ROOT>
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
source <ROBOT_REPO_ROOT>/gello_software/ros2/install/setup.bash
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python train/franka_act_policy_executor.py \
  --policy-path <POLICY_PATH_ON_SERVER> \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --server-address <POLICY_SERVER_IP>:<POLICY_SERVER_PORT> \
  --policy-device <POLICY_TORCH_DEVICE> \
  --actions-per-chunk <ACT_DEPLOY_ACTIONS_PER_CHUNK> \
  --zmq-host <ROBOT_BRIDGE_HOST> \
  --zmq-port <OBSERVATION_ZMQ_PORT> \
  --command-zmq-host <ROBOT_BRIDGE_HOST> \
  --command-zmq-port <COMMAND_ZMQ_PORT> \
  --fps <FPS> \
  --task "<TASK_DESCRIPTION>" \
  --act-chunk-size-threshold <ACT_QUEUE_REFRESH_RATIO> \
  --act-aggregate-ratio-old <ACT_OLD_ACTION_WEIGHT> \
  --action-fusion-mode <FIXED_RATIO_OR_LINEAR_RAMP> \
  --fusion-horizon <FUSION_HORIZON_STEPS> \
  --buffer-horizon <EMPTY_QUEUE_HOLD_STEPS> \
  --run-name <DRY_RUN_LOG_NAME>
```

不加 `--execute` 时只推理和记日志，不向 Franka 发送 Policy Command。Executor 仍会暂时激活 Bridge 以获取 Deployment Observation，退出时自动让 Bridge 回到 standby。

满足以下条件后才能继续：

- Live state/action 维度与数据集一致
- 所有预期相机名存在
- Policy Server 加载了正确 checkpoint
- 输出为有限值，延迟稳定
- 没有其它进程发布 Deployment Joint Target

### 8.7 ACT 实机执行

重启 ACT Executor，加入期望的启动对齐方式、`--limit` 和 `--execute`：

```bash
python train/franka_act_policy_executor.py \
  --policy-path <POLICY_PATH_ON_SERVER> \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --server-address <POLICY_SERVER_IP>:<POLICY_SERVER_PORT> \
  --policy-device <POLICY_TORCH_DEVICE> \
  --actions-per-chunk <ACT_DEPLOY_ACTIONS_PER_CHUNK> \
  --zmq-host <ROBOT_BRIDGE_HOST> \
  --zmq-port <OBSERVATION_ZMQ_PORT> \
  --command-zmq-host <ROBOT_BRIDGE_HOST> \
  --command-zmq-port <COMMAND_ZMQ_PORT> \
  --fps <FPS> \
  --task "<TASK_DESCRIPTION>" \
  --act-chunk-size-threshold <ACT_QUEUE_REFRESH_RATIO> \
  --act-aggregate-ratio-old <ACT_OLD_ACTION_WEIGHT> \
  --action-fusion-mode <FIXED_RATIO_OR_LINEAR_RAMP> \
  --fusion-horizon <FUSION_HORIZON_STEPS> \
  --buffer-horizon <EMPTY_QUEUE_HOLD_STEPS> \
  <ONE_STARTUP_ALIGNMENT_OPTION_OR_NOTHING> \
  --limit \
  --execute
```

将 `<ONE_STARTUP_ALIGNMENT_OPTION_OR_NOTHING>` 替换为以下其中一种：

```bash
--policy-start
```

```bash
--episode-start
```

```bash
--episode-start <EPISODE_INDEX>
```

```bash
--episode-start <EPISODE_INDEX> \
--episode-start-frame-index <EPISODE_LOCAL_FRAME_INDEX>
```

也可以整行删除，直接从第一个普通 Policy Chunk 开始执行。

### 8.8 Diffusion dry-run 和实机执行

改用 Diffusion Executor 和 Diffusion 专用队列参数：

```bash
python train/franka_diffusion_policy_executor.py \
  --policy-path <POLICY_PATH_ON_SERVER> \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --server-address <POLICY_SERVER_IP>:<POLICY_SERVER_PORT> \
  --policy-device <POLICY_TORCH_DEVICE> \
  --actions-per-chunk <DIFFUSION_DEPLOY_ACTIONS_PER_CHUNK> \
  --zmq-host <ROBOT_BRIDGE_HOST> \
  --zmq-port <OBSERVATION_ZMQ_PORT> \
  --command-zmq-host <ROBOT_BRIDGE_HOST> \
  --command-zmq-port <COMMAND_ZMQ_PORT> \
  --fps <FPS> \
  --task "<TASK_DESCRIPTION>" \
  --diffusion-chunk-size-threshold <DIFFUSION_QUEUE_REFRESH_RATIO> \
  --diffusion-aggregate-ratio-old <DIFFUSION_OLD_ACTION_WEIGHT> \
  --action-fusion-mode <FIXED_RATIO_OR_LINEAR_RAMP> \
  --fusion-horizon <FUSION_HORIZON_STEPS> \
  --buffer-horizon <EMPTY_QUEUE_HOLD_STEPS> \
  --run-name <DRY_RUN_LOG_NAME>
```

Dry-run 通过后，使用相同参数重新启动并增加：

```bash
<ONE_STARTUP_ALIGNMENT_OPTION_OR_NOTHING> \
--limit \
--execute
```

### 8.9 启动姿态对齐逻辑

`--policy-start` 与 `--episode-start` 互斥，而且都要求 `--execute`。

- `--policy-start`：用当前 live observation 得到的第一帧、完成 postprocess 后的绝对 action 作为对齐目标。保守移动到位后丢弃已经过期的首个 action chunk，再从对齐后的实时状态重新推理。
- 不带编号的 `--episode-start`：随机选一个数据集 episode，使用指定帧的 `observation.state`。
- `--episode-start <EPISODE_INDEX>`：选择明确的 episode。
- `--episode-start-random-seed <RANDOM_SEED>`：让随机 episode 选择可复现。
- Episode 对齐要求 Robot Host 本地存在 `<ROBOT_DATASET_ROOT>/data/chunk-*/*.parquet`，不需要视频文件。

这些功能通过已经运行的 Deployment Bridge 和 ROS Controller 完成。不要关闭四组部署进程，也不要在这个流程中运行独立的 direct-`pylibfranka` 初始化工具。

Executor 参数约束：

| 参数 | 约束 |
| --- | --- |
| `--actions-per-chunk` | 必须为正数，且不能超过 checkpoint 的最大 chunk size |
| `--act-chunk-size-threshold` / `--diffusion-chunk-size-threshold` | 零到一之间的比例 |
| `--act-aggregate-ratio-old` / `--diffusion-aggregate-ratio-old` | 零到一之间的旧动作权重 |
| `--fusion-horizon` | 正数，表示参与融合的重叠步数 |
| `--buffer-horizon` | 等待新 chunk 时允许保持上一命令的步数 |
| `--policy-start` / `--episode-start` | 最多选择一个，而且只能用于实机执行 |

### 8.10 `--limit` 到底改变什么

不加 `--limit` 时，动作生成行为保持原样。

加上后，每个绝对关节目标都要经过保守 FR3 关节位置、速度、加速度范围检查。超限 segment 会用五次轨迹展开成更多控制步；原本在范围内的 segment 仍是一步。左右臂分别检查，夹爪变化只在拉长 segment 的最后一步发出。

NaN、无穷值和越界关节目标会在发送前被拒绝。生成的限速步骤会以 `action_limit_step` 事件写入 `samples.jsonl`。

`--limit` 是本地关节空间防护，不检查自碰、环境碰撞、笛卡尔速度、外力、相机正确性，也不判断目标动作是否符合任务语义。

### 8.11 日志与停机顺序

Executor 日志位于：

```text
<ROBOT_REPO_ROOT>/outputs/deployment_logs/<EXECUTOR_RUN_NAME>/
```

重点文件为 `metadata.json` 和 `samples.jsonl`。

正常停机顺序：

1. Ctrl-C 停止 Executor；它会请求 Bridge standby 并停止发送命令。
2. 确认左右 Deployment Controller 回到 `inactive`。
3. Ctrl-C 停止 `start_deployment_duo.sh`；脚本再次请求 standby，并反序停止 Bridge、Camera、Gripper 和 Arm。
4. 最后停止 Policy Server。

Executor 异常退出时，先手工请求 standby 再分析：

```bash
ros2 service call /set_deployment_active \
  std_srvs/srv/SetBool \
  "{data: false}"
```

### 8.12 手动启动部署进程，仅用于诊断

正常流程优先使用一键脚本。手动模式的四组顺序为：

```bash
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=example_fr3_duo_config.yaml \
  deployment_mode:=true
```

先启动并等左夹爪初始化，再启动右夹爪：

```bash
ros2 run franka_gripper_manager franka_gripper_client \
  --ros-args -r __node:=franka_gripper_client -r __ns:=/left
```

```bash
ros2 run franka_gripper_manager franka_gripper_client \
  --ros-args -r __node:=franka_gripper_client -r __ns:=/right
```

```bash
ros2 launch franka_realsense_camera_publisher cameras.launch.py \
  config_file:=example_three_cameras.yaml
```

```bash
ros2 launch franka_lerobot_data_bridge bridge.launch.py \
  config_file:=deployment_duo.yaml
```

部署时 Arm Bringup 必须存在：它占有 Franka 连接、发布 Robot State，并加载带 Deployment Gate 的 Controller。部署不需要 GELLO。

## 9. 仅维护用途的工具

### 9.1 独立 Franka 初始化工具

`data_collection/initialize_franka_for_deployment.py` 可以预览或直接移动到某个数据集 episode 状态，或者显式 postprocessed Policy Action。它直接使用 `pylibfranka`，运行前必须停止 ROS Arm Controller。正常部署不使用它。

预览随机 episode 目标：

```bash
python3 data_collection/initialize_franka_for_deployment.py \
  --episode \
  --dataset-root <ROBOT_DATASET_ROOT>
```

预览指定 episode 和帧：

```bash
python3 data_collection/initialize_franka_for_deployment.py \
  --episode <EPISODE_INDEX> \
  --frame-index <EPISODE_LOCAL_FRAME_INDEX> \
  --dataset-root <ROBOT_DATASET_ROOT>
```

只有阅读 `--help` 中的确认要求，并确认没有其它进程占用两台 Franka 后，才考虑加入 `--execute`。

### 9.2 GELLO 主动驱动审计

`gello_software/scripts/test_gello_active_drive.py` 是硬件诊断工具，不属于正常数采或部署。默认 audit 不开扭矩。Active test 必须隔离硬件、确认准确型号，并提供脚本要求的确认短语。

## 10. 常见问题

### Output directory already exists

不带 `--resume` 时 Trainer 拒绝覆盖。先检查目录内容，再选择与原实验匹配的续训、换新目录，或只删除确认无用的空目录。

### `uv: command not found`

```bash
python -m pip install uv
```

### `ModuleNotFoundError: torch`

当前 Conda 环境没有 PyTorch，或激活了错误环境：

```bash
which python
conda info --envs
python -c "import torch; print(torch.__version__)"
```

### `libavutil.so` 或 TorchCodec 加载失败

按环境配置章节，在当前 Conda 环境安装兼容 FFmpeg 并导出 `LD_LIBRARY_PATH`。

### ZMQ 端口被占用

先查清进程归属，不要直接 kill：

```bash
ss -ltnp | grep ':<ZMQ_PORT>'
ps -ef | grep -E '[l]erobot_data_bridge|[f]ranka_.*controller|[g]ello'
```

确认进程用途和使用者之后才能决定是否结束。

### Deployment preflight 提示 ROS Prefix 错误

重新编译，并保证当前仓库 overlay 最后 source：

```bash
cd <ROBOT_REPO_ROOT>/gello_software/ros2
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
colcon build --symlink-install
source install/setup.bash
```

### Executor 一直等待第一帧 ZMQ Packet

确认 `start_deployment_duo.sh` 已到 standby-ready 状态，Executor 能调用 `/set_deployment_active`，Observation 端口一致，并且 Executor 终端 source 了 ROS 环境。

### Policy Server 找不到 checkpoint

`--policy-path` 由 Policy Server 解释。必须传该机器上真实存在的绝对路径，不能传只在 Robot Host 存在的路径。

### State/Action 维度或相机键不一致

不能绕过检查。逐项比较：

- `<ROBOT_DATASET_ROOT>/meta/info.json`
- `<ROBOT_DATASET_ROOT>/meta/real_exp_action_config.json`
- `deployment_duo.yaml`
- Live Bridge 启动输出
- Policy inspect 输出

### Startup alignment 超时

查看最大关节误差和 Deployment Log。超时表示目标没有在 Aligner 时间窗内稳定下来，不代表可以直接删除限制。应检查 Controller 状态、目标可达性、消息频率，以及是否有其它 Publisher 在向同一 Topic 发命令。

### Franka 变红或进入 Reflex Stop

立即停止执行并保留：

- Executor 的 `metadata.json` 和 `samples.jsonl`
- Deployment Stack 日志
- `ros2_control_node` 和 Franka 相关 ROS 日志
- Franka Desk 错误详情
- 最后一帧 Live State 和第一条 Command Target

按时间戳对齐这些记录后再 reset。能 dry-run 复现的问题不要先用实机反复尝试。

## 11. 全流程 Checklist

### 数采

- [ ] 分支、子模块正确，ROS Overlay 已重新编译
- [ ] GELLO 端口、offset 和左右映射已核对
- [ ] Franka FCI 开启，工作空间清空
- [ ] 相机名称与序列号映射稳定
- [ ] `start_collection_duo.sh` 报告全部数采组件 ready
- [ ] Recorder 使用正确的数据集路径、Repo ID、FPS 和任务描述
- [ ] 只在需要时启用 reset-hold
- [ ] 每个 episode 都明确保存或丢弃
- [ ] 数据集 finalize 并通过校验
- [ ] 完整数据集传到训练主机后再次通过校验

### 训练

- [ ] 数据集路径和 Action Metadata 正确
- [ ] 最终 PyTorch 安装能看到目标 GPU
- [ ] W&B 已登录，Project 正确
- [ ] ACT 和 Diffusion 使用不同输出目录
- [ ] 并行训练使用不同 GPU
- [ ] Validation Loss 和 Checkpoint 按预期出现
- [ ] `checkpoints/last/pretrained_model/config.json` 存在

### 部署

- [ ] Policy Server 上已 inspect 数据集元数据和 checkpoint
- [ ] 工作空间清空，Franka 解锁并开启 FCI
- [ ] Deployment preflight 通过
- [ ] 四组 ROS 进程 standby-ready，且没有 GELLO Publisher
- [ ] Robot Host 能访问 Policy Server
- [ ] Executor dry-run 的维度、相机、输出和延迟检查通过
- [ ] 有意选择 Startup Source
- [ ] 第一次实机尝试启用 `--limit`
- [ ] 操作者守在急停旁边
- [ ] 每次测试后保留 Executor 日志
- [ ] 先停 Executor，再停 Deployment Stack 和 Policy Server

# Real-Exp Full Workflow Guide (en-US)

This guide covers the current end-to-end dual-FR3 workflow: repository setup, ROS build, GELLO data collection, optional GELLO reset-and-hold, dataset validation and transfer, ACT and Diffusion training with Weights & Biases, remote policy serving, robot-side dry runs, startup alignment, limited live execution, shutdown, and recovery.

It is written against the current source tree. The one-click scripts and executor options described here are part of this checkout, including:

- `scripts/start_collection_duo.sh`
- `scripts/start_deployment_duo.sh`
- `data_collection/lerobot_collection.py --gello-reset-hold`
- `train/franka_act_policy_executor.py --policy-start | --episode-start [INDEX] [--no-limit]`
- `train/franka_diffusion_policy_executor.py --policy-start | --episode-start [INDEX] [--no-limit]`

## 1. How to use the placeholders

Every value that depends on the user, task, machine, or experiment is written as `<A_CLEAR_DESCRIPTION>`.

Do not paste a command while the angle-bracket placeholders are still present. Replace every placeholder first; otherwise the shell may interpret `<` or `>` as redirection.

Common placeholders:

| Placeholder | Replace with |
| --- | --- |
| `<ROBOT_REPO_ROOT>` | Absolute path to this repository on the Robot Host |
| `<TRAIN_REPO_ROOT>` | Absolute path to this repository on the training host |
| `<POLICY_REPO_ROOT>` | Absolute path to this repository on the policy server |
| `<ROBOT_LEROBOT_ENV>` | Robot-side Conda environment containing LeRobot and executor dependencies |
| `<TRAIN_LEROBOT_ENV>` | Training-side Conda environment containing LeRobot and PyTorch |
| `<POLICY_LEROBOT_ENV>` | Policy-server Conda environment containing LeRobot and PyTorch |
| `<CONDA_ACTIVATION_SCRIPT>` | Conda activation script, such as the host's `bin/activate` path |
| `<FRANKA_WS_SETUP>` | Franka ROS 2 workspace `install/setup.bash` path |
| `<DATASET_REPO_ID>` | LeRobot metadata ID in `namespace/dataset_name` form |
| `<ROBOT_DATASET_ROOT>` | Dataset directory on the Robot Host |
| `<TRAIN_DATASET_ROOT>` | Copy of the same dataset on the training host |
| `<TASK_DESCRIPTION>` | Natural-language task instruction stored with samples and sent to the policy |
| `<ACT_OUTPUT_DIR>` | Training output directory dedicated to ACT |
| `<DIFFUSION_OUTPUT_DIR>` | Training output directory dedicated to Diffusion |
| `<POLICY_PATH_ON_SERVER>` | `.../checkpoints/last/pretrained_model` path visible on the policy server |
| `<POLICY_SERVER_IP>` | Robot-reachable policy-server IP address |
| `<POLICY_SERVER_PORT>` | gRPC policy-server port |
| `<ROBOT_SSH_USER>`, `<ROBOT_SSH_HOST>`, `<ROBOT_SSH_PORT>` | Robot Host SSH login fields |
| `<TRAIN_SSH_USER>`, `<TRAIN_SSH_HOST>`, `<TRAIN_SSH_PORT>` | Training host SSH login fields |
| `<POLICY_SSH_USER>`, `<POLICY_SSH_HOST>`, `<POLICY_SSH_PORT>` | Policy-server SSH login fields |
| `<FPS>` | One control/recording frequency used consistently across collection, training metadata, server, and executor |

## 2. Machine roles and process boundaries

### Robot Host

The Robot Host is physically connected to both Frankas, both GELLOs, both grippers, and all cameras. It runs:

- ROS 2 hardware and state publishers
- collection or deployment controllers
- gripper clients
- camera publishers
- the ROS-to-ZMQ bridge
- the LeRobot recorder during data collection
- the ACT or Diffusion executor during deployment

### Training Host

The training host stores a copied dataset and trains ACT and Diffusion checkpoints. Training can run in separate `tmux` sessions on separate GPUs.

### Policy Server

The policy server loads the checkpoint requested by the Robot Host, receives observations over gRPC, performs inference, and returns action chunks. The checkpoint path passed to an executor must therefore exist on the policy server, even when the executor itself runs on the Robot Host.

The training host and policy server may be the same physical machine, but their roles and terminal sessions remain separate.

### Current process map and changes

| Process or supervisor | Host | Current behavior/change |
| --- | --- | --- |
| `start_collection_duo.sh` | Robot | Starts and supervises five ROS collection groups; the recorder intentionally stays separate |
| GELLO publisher | Robot | Used only for teleoperation collection; its parameters also support recorder-managed active reset/hold |
| Collection arm bringup | Robot | Runs the normal teleoperation controller path without deployment mode |
| Collection bridge | Robot | Uses `example_duo.yaml` and publishes synchronized recording packets |
| `lerobot_collection.py` | Robot | Adds `--gello-reset-hold` and `r` while preserving the old behavior when the flag is absent |
| `start_deployment_duo.sh` | Robot | Preflights, starts, readiness-checks, monitors, and safely tears down four deployment groups |
| Deployment arm bringup | Robot | Uses `deployment_mode:=true`; loads deployment controllers inactive until bridge activation and a real command |
| Gripper clients | Robot | Start left then right in both one-click flows to avoid concurrent initialization timeouts |
| Deployment bridge | Robot | Uses `deployment_duo.yaml`, starts in standby, publishes observations, accepts commands, and gates controllers |
| Policy server | GPU host | Loads the server-visible checkpoint requested by the executor; supports deterministic or stochastic Diffusion noise |
| ACT/Diffusion executor | Robot | Auto-activates/deactivates the bridge, applies joint time-stretching by default, and supports startup alignment, action fusion, bounded last-command hold, and deployment logs |

### SSH command templates

These commands do not depend on `~/.ssh/config`:

```bash
ssh -p <ROBOT_SSH_PORT> <ROBOT_SSH_USER>@<ROBOT_SSH_HOST>
```

```bash
ssh -p <TRAIN_SSH_PORT> <TRAIN_SSH_USER>@<TRAIN_SSH_HOST>
```

```bash
ssh -p <POLICY_SSH_PORT> <POLICY_SSH_USER>@<POLICY_SSH_HOST>
```

When training and policy serving use the same host, open separate SSH terminals or `tmux` sessions for the two roles.

## 3. Safety rules

1. Unlock both Frankas and enable FCI only after the workspaces are clear.
2. Keep an operator at the emergency stop during first live execution.
3. Never run the collection controller stack and deployment controller stack at the same time.
4. Never start a GELLO publisher during policy deployment.
5. Never run direct `pylibfranka` initialization while ROS arm controllers have the robot connection.
6. Run the executor without `--execute` first. Confirm state/action dimensions, camera keys, latency, and finite policy outputs.
7. Joint-space limiting and time-stretching are enabled by default. `--no-limit` disables them but does not remove the remaining validity checks, and neither mode proves Cartesian collision safety or task safety.
8. Stop an unexpected motion immediately. Do not repeatedly clear a Franka reflex without identifying the command that caused it.

## 4. Repository and environment setup

### 4.1 Clone and initialize submodules

Run on every machine that needs the source tree:

```bash
git clone --recurse-submodules <REAL_EXP_GIT_URL> <REPO_DESTINATION>
cd <REPO_DESTINATION>
git submodule update --init --recursive
```

### 4.2 Build the Robot Host ROS overlay

Use system ROS Python, not the LeRobot Conda environment:

```bash
cd <ROBOT_REPO_ROOT>/gello_software/ros2
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
colcon build --symlink-install
source install/setup.bash
```

Rebuild after changing anything under `gello_software/ros2/`. Source the overlay again in every new ROS terminal.

### 4.3 Create a LeRobot environment

Run separately on each host that needs Python inference, recording, or training:

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

Verify the final environment after the editable install, because dependency resolution can replace an earlier PyTorch build:

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'no cuda')"
python -c "import lerobot, wandb, zmq, pyarrow; print('Python environment OK')"
```

If TorchCodec reports a missing `libavutil.so`, install a compatible FFmpeg package in the active environment and make its libraries visible:

```bash
conda install -c conda-forge "ffmpeg=<COMPATIBLE_FFMPEG_MAJOR>.*" -y
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

### 4.4 Source the correct Robot Host environment

ROS stack terminal:

```bash
cd <ROBOT_REPO_ROOT>
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
source <ROBOT_REPO_ROOT>/gello_software/ros2/install/setup.bash
```

Recorder or executor terminal:

```bash
cd <ROBOT_REPO_ROOT>
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
source <ROBOT_REPO_ROOT>/gello_software/ros2/install/setup.bash
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

The executor needs the ROS environment because it calls `/set_deployment_active` through the `ros2` CLI.

## 5. Data collection

### 5.1 What the dataset stores

The dual-arm collection bridge and recorder produce:

- `observation.state`: measured left arm, left gripper, right arm, and right gripper state
- `action`: next-sample absolute joint targets plus gripper commands
- `observation.images.cam_left`
- `observation.images.cam_front`
- `observation.images.cam_right`
- `meta/real_exp_action_config.json`: action representation and dimensional contract

The expected dual-arm layout is `[left_arm_7, left_gripper_1, right_arm_7, right_gripper_1]`. Do not assume this layout blindly; verify the generated metadata before training.

### 5.2 Hardware preparation

1. Confirm both GELLO USB device paths used by `gello_duo.yaml`.
2. Confirm left and right GELLO joint offsets after reconnection or mechanical changes.
3. Confirm both Franka IPs and namespaces in `example_fr3_duo_config.yaml`.
4. Confirm stable camera serial-to-name assignment in `example_three_cameras.yaml`.
5. Unlock both Frankas, enable FCI, clear both arm workspaces, and power both grippers.
6. Ensure the observation ZMQ port is not occupied by an old bridge.

Check GELLO and Franka state topics after the publishers are running:

```bash
ros2 topic echo --once /left/gello/joint_states
ros2 topic echo --once /right/gello/joint_states
ros2 topic echo --once /left/franka/joint_states
ros2 topic echo --once /right/franka/joint_states
```

### 5.3 Start the collection ROS stack with one command

Terminal 1, system ROS environment:

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_collection_duo.sh
```

Optional supervisor overrides are environment variables, not command-line flags:

```bash
READY_TIMEOUT=<STARTUP_TIMEOUT_SECONDS> \
COLLECTION_LOG_DIR=<COLLECTION_STACK_LOG_DIR> \
COLLECTION_ZMQ_PORT=<COLLECTION_OBSERVATION_PORT> \
bash scripts/start_collection_duo.sh
```

If the collection port is overridden, `example_duo.yaml` must be updated and the ROS overlay rebuilt so the bridge listens on the same port.

The script starts five groups in dependency order:

| Order | Group | Current command/configuration | Purpose |
| --- | --- | --- | --- |
| 1 | GELLO | `franka_gello_state_publisher` with `gello_duo.yaml` | Publishes left/right teleoperator joint targets |
| 2 | Arms | `franka_fr3_arm_controllers` with `example_fr3_duo_config.yaml` | Normal teleoperation controller path; no deployment mode |
| 3 | Grippers | Left client, then right client | Sequential homing avoids concurrent action startup timeouts |
| 4 | Cameras | `example_three_cameras.yaml` | Publishes the three named RGB streams |
| 5 | Bridge | `example_duo.yaml` | Publishes synchronized collection packets over ZMQ |

The script checks that the collection port is free, waits for real topic messages instead of topic names alone, monitors every child process, writes logs under a timestamp-named directory below `log/collection/`, and stops the complete process tree in reverse order on Ctrl-C.

It intentionally does not start `lerobot_collection.py`. The recorder remains in a second Conda terminal so recording controls stay visible and a recorder crash cannot tear down the hardware stack.

### 5.4 Start the recorder

Terminal 2, LeRobot Conda environment:

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

Recorder controls:

| Input | Action |
| --- | --- |
| `s` + Enter | Start a new episode |
| `e` + Enter | End and save the current episode |
| `d` + Enter | Discard the current episode buffer |
| `q` + Enter | Quit, finalize metadata/statistics, and save pending frames if present |

The recorder resumes a compatible existing dataset. If the live stream contract differs, it creates a new sibling directory instead of silently appending incompatible features.

### 5.5 Optional integrated GELLO reset and hold

Add `--gello-reset-hold` only when the active reset feature is needed:

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

This adds `r` + Enter:

1. `r` actively maps the known Franka initial joint state back into each GELLO's raw joint coordinates.
2. Both GELLOs move toward that target and remain torque-held.
3. `s` starts recording and arms automatic release.
4. Torque is disabled only after real GELLO joint motion is detected, so the operator can take over without an immediate target jump.
5. `q`, Ctrl-C, or recorder cleanup disables GELLO torque.

The helper is managed automatically by the recorder and uses the running GELLO publisher parameter services. Do not run another active GELLO drive tool at the same time. `r` is rejected while an episode is being recorded.

### 5.6 Manual collection startup, for diagnosis only

The one-click script is preferred. If a group must be isolated, use the same order and commands:

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

After the left client finishes initialization:

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

Do not add `deployment_mode:=true` and do not use `deployment_duo.yaml` during collection.

### 5.7 End collection safely

1. Save or discard the active episode in the recorder.
2. Press `q` + Enter and wait for dataset finalization.
3. Press Ctrl-C in `start_collection_duo.sh`.
4. Confirm the ZMQ port and Franka connections were released before starting another stack.

## 6. Dataset validation, editing, and transfer

### 6.1 Validate immediately after collection

```bash
cd <ROBOT_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <ROBOT_LEROBOT_ENV>

python data_collection/validate_dataset.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --verbose
```

The validator checks episode/frame continuity, metadata totals, state/action dimensions, timestamps, videos, gripper bounds, and suspicious arm action jumps. Treat warnings as investigation targets, not automatic proof that smoothing is required.

If OpenCV is unavailable and only the physical video frame-count test must be skipped:

```bash
python data_collection/validate_dataset.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --skip-video-frames
```

### 6.2 Delete bad episodes without editing Parquet manually

Preview first:

```bash
python data_collection/delete_lerobot_episode.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --episode-indices <SPACE_SEPARATED_EPISODE_INDICES> \
  --dry-run
```

Write an edited copy:

```bash
python data_collection/delete_lerobot_episode.py \
  --dataset-root <ROBOT_DATASET_ROOT> \
  --episode-indices <SPACE_SEPARATED_EPISODE_INDICES> \
  --output-dir <EDITED_DATASET_ROOT>
```

Validate the edited result again. Keep the raw dataset immutable until the processed copy has passed validation.

### 6.3 Transfer to the training host

When the Robot Host can reach the training host directly:

```bash
rsync -avhP \
  -e "ssh -p <TRAIN_SSH_PORT>" \
  <ROBOT_DATASET_ROOT>/ \
  <TRAIN_SSH_USER>@<TRAIN_SSH_HOST>:<TRAIN_DATASET_ROOT>/
```

If direct routing is unavailable, copy through a workstation in two steps. Preserve the complete dataset root, including `data/`, `videos/`, and `meta/`.

Validate the transferred copy on the training host:

```bash
cd <TRAIN_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>

python data_collection/validate_dataset.py \
  --dataset-root <TRAIN_DATASET_ROOT>
```

## 7. Train ACT and Diffusion with W&B

### 7.1 Authenticate W&B once per training account

```bash
source <CONDA_ACTIVATION_SCRIPT>
conda activate <TRAIN_LEROBOT_ENV>
wandb login
```

Do not pass `--disable-wandb` when online logging is required.

### 7.2 Train ACT in `tmux`

```bash
tmux new -s <ACT_TRAIN_TMUX_SESSION>
```

Inside the session:

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

Detach with Ctrl-B, then D. Reattach later:

```bash
tmux attach -t <ACT_TRAIN_TMUX_SESSION>
```

### 7.3 Train Diffusion in a second `tmux` session

```bash
tmux new -s <DIFFUSION_TRAIN_TMUX_SESSION>
```

Inside the session:

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

Use different GPU IDs when ACT and Diffusion run concurrently. `CUDA_VISIBLE_DEVICES` remaps the selected physical GPU to `cuda:0` inside that process.

### 7.4 Resume an interrupted run

Use the same dataset, policy type, structural policy arguments, and output directory. Add `--resume`, and set `--steps` to the intended final total:

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

Do not use `--resume` on an empty or unrelated output directory. Without `--resume`, the trainer intentionally refuses to overwrite an existing output directory.

### 7.5 Confirm training output

The deployable checkpoint is:

```text
<TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model
```

Check it before deployment:

```bash
test -f <TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model/config.json
ls -lh <TRAIN_OUTPUT_DIR>/checkpoints/last/pretrained_model
```

### 7.6 Copy artifacts when the policy server is a different host

Copy the complete training output so checkpoint links and processor files remain together:

```bash
rsync -avhP \
  -e "ssh -p <POLICY_SSH_PORT>" \
  <TRAIN_OUTPUT_DIR>/ \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST>:<POLICY_OUTPUT_DIR>/
```

Copy at least the dataset metadata required by `inspect`:

```bash
ssh -p <POLICY_SSH_PORT> \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST> \
  "mkdir -p <POLICY_SERVER_DATASET_ROOT>/meta"

rsync -avhP \
  -e "ssh -p <POLICY_SSH_PORT>" \
  <TRAIN_DATASET_ROOT>/meta/ \
  <POLICY_SSH_USER>@<POLICY_SSH_HOST>:<POLICY_SERVER_DATASET_ROOT>/meta/
```

After transfer, `<POLICY_PATH_ON_SERVER>` should resolve to `<POLICY_OUTPUT_DIR>/checkpoints/last/pretrained_model` on that host.

## 8. Deploy a trained policy

Deployment consists of three independent layers:

1. Robot Host ROS deployment stack: four supervised hardware groups.
2. Policy server: one inference process on the GPU host.
3. Robot Host executor: one ACT or Diffusion process that connects the two layers.

GELLO is not part of deployment.

### 8.1 Collection versus deployment configuration

| Item | Data collection | Policy deployment |
| --- | --- | --- |
| GELLO publisher | Required | Forbidden |
| Arm launch | Normal teleoperation mode | `deployment_mode:=true` |
| Deployment joint controllers | Not used | Loaded inactive, activated by bridge only after commands arrive |
| Camera YAML | `example_three_cameras.yaml` | `example_three_cameras.yaml` |
| Bridge YAML | `example_duo.yaml` | `deployment_duo.yaml` |
| ZMQ observation stream | Collection samples | Live policy observations |
| ZMQ command stream | Not used | Policy actions back to bridge |
| Initial bridge state | Active collection stream | `STANDBY` |

### 8.2 Preflight the Robot Host without connecting to Franka

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_deployment_duo.sh --preflight-only
```

Preflight verifies:

- ROS setup files and required commands
- installed ROS packages resolve to this checkout's overlay
- arm, gripper, camera, and bridge YAML contracts
- deployment bridge mode, state source, topics, services, and ports
- no old GELLO, bridge, or controller stack conflicts
- observation and command ports are free

Fix every preflight error before starting hardware.

### 8.3 Start all four deployment ROS groups

Terminal 1 on the Robot Host, system ROS environment:

```bash
cd <ROBOT_REPO_ROOT>
bash scripts/start_deployment_duo.sh
```

The script starts:

1. Franka hardware, robot/joint state publishers, and inactive deployment controllers using `example_fr3_duo_config.yaml` plus `deployment_mode:=true`.
2. Left and right gripper clients sequentially.
3. Three camera publishers using `example_three_cameras.yaml`.
4. The bridge using `deployment_duo.yaml`, observation ZMQ, command ZMQ, and `/set_deployment_active`.

It does not start the policy server or executor. Continue only after it prints:

```text
All four deployment ROS groups are ready in STANDBY mode
```

Logs are written below a timestamp-named directory under `log/deployment_stack/`. Runtime endpoints are checked every few seconds; three consecutive failures stop the supervised stack.

Optional environment overrides must be supplied before the command:

```bash
READY_TIMEOUT=<STARTUP_TIMEOUT_SECONDS> \
DEPLOYMENT_LOG_DIR=<DEPLOYMENT_STACK_LOG_DIR> \
DEPLOYMENT_OBSERVATION_PORT=<OBSERVATION_ZMQ_PORT> \
DEPLOYMENT_COMMAND_PORT=<COMMAND_ZMQ_PORT> \
bash scripts/start_deployment_duo.sh
```

If ports are overridden, the installed `deployment_duo.yaml` must contain matching values or preflight will reject the configuration.

### 8.4 Inspect the checkpoint on the policy server

```bash
cd <POLICY_REPO_ROOT>
source <CONDA_ACTIVATION_SCRIPT>
conda activate <POLICY_LEROBOT_ENV>
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python train/deploy_lerobot_policy.py inspect \
  --policy-path <POLICY_PATH_ON_SERVER> \
  --dataset-root <POLICY_SERVER_DATASET_ROOT>
```

Confirm policy type, maximum actions per chunk, FPS, state/action dimensions, camera keys, and action representation. The dataset metadata copy is required for inspection even though inference loads the checkpoint itself.

### 8.5 Start the policy server

Terminal 2 on the policy server:

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

The server command does not include a checkpoint path. The executor sends `<POLICY_PATH_ON_SERVER>`, policy type, chunk length, and device when it connects. A single server can therefore load the requested ACT or Diffusion checkpoint for the active executor.

For stochastic Diffusion sampling, replace the fixed-seed option with `--disable-diffusion-fixed-noise`.

Verify reachability from the Robot Host:

```bash
nc -vz <POLICY_SERVER_IP> <POLICY_SERVER_PORT>
```

### 8.6 ACT dry run

Terminal 3 on the Robot Host, LeRobot Conda plus ROS environment:

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

Without `--execute`, predictions are logged but no policy commands are sent to Franka. The executor still temporarily activates the bridge so it can receive deployment observations, then returns the bridge to standby when it exits.

Proceed only after confirming:

- live state and action dimensions match the dataset
- all expected camera names are present
- the policy server loads the intended checkpoint
- outputs are finite and latency is stable
- no unexpected process is publishing deployment joint targets

### 8.7 ACT live execution

Restart the ACT executor and add the desired startup mode and `--execute`. The limiter is already enabled:

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
  --execute
```

Replace `<ONE_STARTUP_ALIGNMENT_OPTION_OR_NOTHING>` with exactly one of:

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

Or remove that line entirely to start immediately from the first normal policy chunk.

### 8.8 Diffusion dry run and live execution

Use the Diffusion executor and Diffusion-specific queue controls:

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

After the dry run passes, restart with the same arguments and add:

```bash
<ONE_STARTUP_ALIGNMENT_OPTION_OR_NOTHING> \
--execute
```

### 8.9 Startup alignment behavior

`--policy-start` and `--episode-start` are mutually exclusive and require `--execute`.

- `--policy-start` uses the first postprocessed absolute action predicted from the current live observation as the alignment target. It approaches conservatively, discards the now-stale first action chunk, and requests fresh inference from the aligned state.
- `--episode-start` without an index randomly selects a dataset episode and uses its requested frame's `observation.state`.
- `--episode-start <EPISODE_INDEX>` selects a specific episode.
- `--episode-start-random-seed <RANDOM_SEED>` makes random episode selection reproducible.
- Episode startup requires local Parquet files under `<ROBOT_DATASET_ROOT>/data/chunk-*/*.parquet`; videos are not required.

These options run through the already-active deployment bridge and ROS controllers. Do not stop the four deployment groups and do not run the standalone direct-`pylibfranka` initializer for this path.

Executor parameter constraints:

| Parameter | Constraint |
| --- | --- |
| `--actions-per-chunk` | Positive and no greater than the checkpoint's maximum chunk size |
| `--act-chunk-size-threshold` / `--diffusion-chunk-size-threshold` | Ratio from zero through one |
| `--act-aggregate-ratio-old` / `--diffusion-aggregate-ratio-old` | Old-action weight from zero through one |
| `--fusion-horizon` | Positive number of overlap steps |
| `--buffer-horizon` | Number of last-command hold steps allowed while waiting for a new chunk |
| `--policy-start` / `--episode-start` | At most one; live execution only |

### 8.10 Default limiter and `--no-limit`

Joint position, velocity, and acceleration limiting is enabled by default for both executors. Omitting both limiter flags still creates a `PolicyActionLimiter` and time-stretches over-limit segments.

`--limit` remains accepted for backward compatibility and explicitly selects the same default behavior. Only `--no-limit` disables time-stretching and sends valid targets directly.

With the default limiter active, each absolute arm target is checked against the configured conservative FR3 joint position, velocity, and acceleration envelope. An over-limit segment is expanded into additional control steps using a quintic trajectory; a segment already inside the envelope remains one step. The two arms are evaluated independently, and gripper changes are emitted only on the final stretched step.

The executor rejects NaN, infinity, and out-of-range arm targets before they are sent. Generated limiter steps are recorded in `samples.jsonl` as `action_limit_step` events.

The limiter is a local joint-space safeguard. It does not check self-collision, environment collision, Cartesian velocity, external forces, camera correctness, or whether a target is semantically sensible. Use `--no-limit` only for a deliberate comparison or diagnosis.

### 8.11 Logs and shutdown

Executor logs are created under:

```text
<ROBOT_REPO_ROOT>/outputs/deployment_logs/<EXECUTOR_RUN_NAME>/
```

Important files include `metadata.json` and `samples.jsonl`.

Normal shutdown order:

1. Ctrl-C the executor. It requests bridge standby and stops sending commands.
2. Confirm both deployment controllers return to `inactive`.
3. Ctrl-C `start_deployment_duo.sh`. It requests standby again and stops bridge, cameras, grippers, and arms in reverse order.
4. Stop the policy server.

If an executor dies unexpectedly, manually request standby before diagnosis:

```bash
ros2 service call /set_deployment_active \
  std_srvs/srv/SetBool \
  "{data: false}"
```

### 8.12 Manual deployment startup, for diagnosis only

The one-click script is preferred. The exact four-group order is:

```bash
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py \
  robot_config_file:=example_fr3_duo_config.yaml \
  deployment_mode:=true
```

Start the left gripper client, wait for initialization, then the right client:

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

The arm bringup is required during deployment: it owns the Franka connection, publishes robot state, and loads the deployment-gated controllers. GELLO is not required.

## 9. Maintenance-only tools

### 9.1 Standalone Franka initialization

`data_collection/initialize_franka_for_deployment.py` can preview or directly move to a dataset episode state or an explicit postprocessed policy action. It uses direct `pylibfranka`, so stop ROS arm controllers first. It is not part of normal deployment.

Preview a random episode target:

```bash
python3 data_collection/initialize_franka_for_deployment.py \
  --episode \
  --dataset-root <ROBOT_DATASET_ROOT>
```

Preview a specific episode target:

```bash
python3 data_collection/initialize_franka_for_deployment.py \
  --episode <EPISODE_INDEX> \
  --frame-index <EPISODE_LOCAL_FRAME_INDEX> \
  --dataset-root <ROBOT_DATASET_ROOT>
```

Use `--execute` only after reading the script's confirmation requirement with `--help` and confirming no other process owns either Franka.

### 9.2 GELLO active-drive audit

`gello_software/scripts/test_gello_active_drive.py` is a hardware diagnostic, not a collection or deployment process. Its default audit leaves torque off. Active tests require physical isolation, exact model confirmation, and the script's explicit confirmation phrase.

## 10. Troubleshooting

### Output directory already exists

The trainer refuses overwrite when `--resume` is absent. Inspect the directory. Resume the matching run, choose a new output directory, or remove only a verified disposable/empty directory.

### `uv: command not found`

```bash
python -m pip install uv
```

### `ModuleNotFoundError: torch`

The active Conda environment does not contain PyTorch, or the wrong environment is active. Check:

```bash
which python
conda info --envs
python -c "import torch; print(torch.__version__)"
```

### `libavutil.so` or TorchCodec load failure

Install a compatible FFmpeg in the active Conda environment and export `LD_LIBRARY_PATH` as shown in environment setup.

### ZMQ port already in use

Find the owner before killing anything:

```bash
ss -ltnp | grep ':<ZMQ_PORT>'
ps -ef | grep -E '[l]erobot_data_bridge|[f]ranka_.*controller|[g]ello'
```

Do not terminate a process until its owner and purpose are known.

### Deployment preflight reports the wrong ROS prefix

Rebuild and source the local overlay last:

```bash
cd <ROBOT_REPO_ROOT>/gello_software/ros2
source /opt/ros/humble/setup.bash
source <FRANKA_WS_SETUP>
colcon build --symlink-install
source install/setup.bash
```

### Executor waits for the first ZMQ packet

Check that `start_deployment_duo.sh` reached standby-ready state and that the executor can call `/set_deployment_active`. Confirm matching observation ports and ROS environment.

### Policy server cannot find the checkpoint

`--policy-path` is interpreted by the policy server. Use an absolute path that exists on that machine, not a Robot Host-only path.

### State/action dimension or camera-key mismatch

Do not bypass the check. Compare:

- `<ROBOT_DATASET_ROOT>/meta/info.json`
- `<ROBOT_DATASET_ROOT>/meta/real_exp_action_config.json`
- `deployment_duo.yaml`
- live bridge startup output
- policy inspection output

### Startup alignment times out

Inspect the maximum joint error and deployment logs. A timeout means the target did not settle within the aligner's configured window; it is not permission to remove limits blindly. Check controller state, target reachability, message rate, and whether another publisher is commanding the same topics.

### Franka turns red or enters reflex stop

Stop execution and preserve:

- executor `metadata.json` and `samples.jsonl`
- deployment stack logs
- ROS logs for `ros2_control_node` and Franka nodes
- Franka Desk error details
- the last live state and first commanded target

Compare the failure timestamp across these sources before resetting. Reproduce first in dry-run whenever possible.

## 11. End-to-end checklists

### Collection checklist

- [ ] Correct branch/submodules and rebuilt ROS overlay
- [ ] GELLO ports, offsets, and left/right mapping verified
- [ ] Franka FCI enabled and workspaces clear
- [ ] Camera name/serial mapping verified
- [ ] `start_collection_duo.sh` reports all collection components ready
- [ ] Recorder uses the intended dataset root, repo ID, FPS, and task string
- [ ] Optional reset-hold enabled only when needed
- [ ] Every episode saved or discarded intentionally
- [ ] Dataset finalized and validation passed
- [ ] Complete dataset copied and revalidated on the training host

### Training checklist

- [ ] Correct dataset root and action metadata
- [ ] Final PyTorch build sees the intended GPU
- [ ] W&B authenticated and correct project selected
- [ ] ACT and Diffusion use separate output directories
- [ ] Concurrent runs use separate GPUs
- [ ] Validation loss and checkpoints appear at expected intervals
- [ ] `checkpoints/last/pretrained_model/config.json` exists

### Deployment checklist

- [ ] Dataset metadata and checkpoint inspected on the policy server
- [ ] Robot workspace clear, Frankas unlocked, FCI enabled
- [ ] Deployment preflight passed
- [ ] Four ROS groups ready in standby; no GELLO publisher running
- [ ] Policy server reachable from Robot Host
- [ ] Executor dry run passed dimensions, cameras, output, and latency checks
- [ ] Startup source selected intentionally
- [ ] Default limiter retained for live trials; `--no-limit` used only for an intentional diagnosis
- [ ] Operator at emergency stop
- [ ] Executor logs preserved after every trial
- [ ] Executor stopped before deployment stack and policy server

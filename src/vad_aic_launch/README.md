# VAD AIC Launch

This package provides integrated launch files for VAD planner that combines AWSIM dummy publisher and TensorRT VAD for autonomous driving.

## Overview

The VAD AIC Launch package integrates the following components:
- **autoware_tensorrt_vad**: VAD for trajectory planning
- **dummy_publisher**: dummy camera and tf topic publisher for VAD

## Build

```bash
colcon build --symlink-install --packages-up-to vad_aic_launch
source install/setup.bash
```

## Usage

### Launch

```bash
ros2 launch vad_aic_launch vad_aic.launch.xml use_sim_time:=true
```

### Available Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Whether to use simulated time |

Check available arguments:
```bash
ros2 launch vad_aic_launch vad_aic.launch.xml --show-args
```

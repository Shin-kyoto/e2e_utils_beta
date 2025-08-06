# VLM Dual Planner Launch

This package provides integrated launch files for VLM-based dual planning system that combines AWSIM dummy publisher and TensorRT VAD (Vector Autoregressive Decoder) for autonomous driving.

## Overview

The VLM Dual Planner Launch package integrates the following components:
- **dummy_publisher**: AWSIM topic publisher for camera and TF data simulation
- **autoware_tensorrt_vad**: TensorRT-accelerated VAD for trajectory planning

## Features

- Single launch file for integrated system startup
- Configurable simulation parameters
- Support for both simulation and real-time modes
- Camera data simulation with multi-camera setup
- TF static publishing for sensor calibration

## Usage

### Basic Launch

```bash
ros2 launch vlm_dual_planner_launch vlm_dual.launch.xml use_sim_time:=true
```

### Available Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Whether to use simulation time |


## Build

```bash
colcon build --symlink-install --packages-up-to vlm_dual_planner_launch
source install/setup.bash
```

## Testing

Check available arguments:
```bash
ros2 launch vlm_dual_planner_launch vlm_dual.launch.xml --show-args
```

Print launch description without running:
```bash
ros2 launch vlm_dual_planner_launch vlm_dual.launch.xml --print-description
```

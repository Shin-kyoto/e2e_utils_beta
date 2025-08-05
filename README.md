# Automotive AI Challenge e2e_utils_beta

- This repository is intended to be used together with [aichallenge-2025](https://github.com/AutomotiveAIChallenge/aichallenge-2025/tree/main).
- It aims to provide samples and references for those who want to create planners using machine learning (ML).

## e2e_utils_beta Setup Instructions

### 1. Setup environment

- Please follow [this document](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-development-environment) and setup the environment with TensorRT.

```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```

- After execute the blow command, the command asked the questions. Please answer them.

```bash
./setup-dev-env.sh
```

### 2. Clone the repository

```bash
git clone https://github.com/Shin-kyoto/e2e_utils_beta.git
```

### 3. Import dependencies
```bash
sh script/setup.sh
```

### 4. Build
```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad
```

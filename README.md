# Automotive AI Challenge e2e_utils_beta

- This repository is intended to be used together with [aichallenge-2025](https://github.com/AutomotiveAIChallenge/aichallenge-2025/tree/main).
- It aims to provide samples and references for those who want to create planners using machine learning (ML).

## e2e_utils_beta Setup Instructions

### 1. Clone the repository

```bash
git clone https://github.com/AutomotiveAIChallenge/aichallenge-2025.git
cd aichallenge/workspace/src
git clone https://github.com/Shin-kyoto/e2e_utils_beta.git
```

### 2. Import dependencies
```bash
cd aichallenge/workspace/src/e2e_utils_beta
sh script/setup.sh
```

### 3. Build
```bash
cd aichallenge/workspace
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad
```

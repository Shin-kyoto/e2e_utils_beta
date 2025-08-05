# Automotive AI Challenge e2e_utils_beta

- This repository is intended to be used together with [aichallenge-2025](https://github.com/AutomotiveAIChallenge/aichallenge-2025/tree/main).
- It aims to provide samples and references for those who want to create planners using machine learning (ML).

## e2e_utils_beta Setup Instructions

### 1. Clone the repository

Please clone this repository in [aichallenge/workspace/src](https://github.com/AutomotiveAIChallenge/aichallenge-2025/tree/main/aichallenge/workspace/src)

```bash
cd ~/aichallenge-2025/aichallenge/workspace/src
git clone https://github.com/Shin-kyoto/e2e_utils_beta.git
cd e2e_utils_beta
```

### 2. Import dependencies
```bash
mkdir external
vcs import external < autoware.repos --shallow
```

### 3. Cleanup unnecessary directories
```bash
python script/cleanup_external.py
```

### 4. Build
```bash
cd ../../
colcon build --symlink-install --packages-up-to autoware_tensorrt_vad
```

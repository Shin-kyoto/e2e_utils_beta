#!/usr/bin/env python3
"""
Cleanup script for external/universe directory.
Keeps only the following directories:
  universe/perception/autoware_tensorrt_common
  universe/perception/autoware_tensorrt_plugins
  universe/common/autoware_cuda_dependency_meta
  universe/sensing/autoware_cuda_utils
Removes all other contents under universe.
"""
import shutil
from pathlib import Path

# Get the path to the universe directory (relative to this script)
SCRIPT_DIR = Path(__file__).resolve().parent
UNIVERSE_DIR = (SCRIPT_DIR / '../external/universe').resolve()

# Directories to keep (relative to UNIVERSE_DIR)
KEEP_RELATIVE = [
    'perception/autoware_tensorrt_common',
    'perception/autoware_tensorrt_plugins',
    'common/autoware_cuda_dependency_meta',
    'sensing/autoware_cuda_utils',
]
# Also keep their parent directories (e.g., 'perception', 'common', 'sensing')
PARENT_RELATIVE = set(Path(rel).parts[0] for rel in KEEP_RELATIVE)
KEEP_TOPLEVEL = set(PARENT_RELATIVE)
KEEP_PATHS = [UNIVERSE_DIR / rel for rel in KEEP_RELATIVE]

# Remove all top-level items in universe except those in KEEP_TOPLEVEL
for item in UNIVERSE_DIR.iterdir():
    if item.name in KEEP_TOPLEVEL:
        continue
    print(f"Removing {item}")
    if item.is_dir():
        shutil.rmtree(item)
    else:
        item.unlink()

# For each kept top-level directory, remove subdirectories/files not in KEEP_PATHS
for parent in KEEP_TOPLEVEL:
    parent_dir = UNIVERSE_DIR / parent
    if not parent_dir.is_dir():
        continue
    # List all direct children
    for child in parent_dir.iterdir():
        # If this child is not in the keep list, remove it
        if not any(child == kp for kp in KEEP_PATHS):
            print(f"Removing {child}")
            if child.is_dir():
                shutil.rmtree(child)
            else:
                child.unlink()

#!/usr/bin/env python3
"""
Script to cleanup external/universe using the cleanup_universe module.
"""
from pathlib import Path

from cleanup_universe import cleanup_universe
from cleanup_core import cleanup_core

if __name__ == "__main__":
    SCRIPT_DIR = Path(__file__).resolve().parent

    # Clean up universe
    UNIVERSE_DIR = (SCRIPT_DIR / '../src/external/universe').resolve()
    KEEP_RELATIVE_UNIVERSE = [
        'perception/autoware_tensorrt_common',
        'perception/autoware_tensorrt_plugins',
        'common/autoware_cuda_dependency_meta',
        'sensing/autoware_cuda_utils',
    ]
    cleanup_universe(UNIVERSE_DIR, KEEP_RELATIVE_UNIVERSE)

    # Clean up core
    CORE_DIR = (SCRIPT_DIR / '../src/external/core').resolve()
    KEEP_RELATIVE_CORE = [
        'autoware_internal_msgs/autoware_internal_planning_msgs',
        'autoware_msgs/autoware_common_msgs',
        'autoware_msgs/autoware_perception_msgs',
        'autoware_msgs/autoware_planning_msgs',
        'autoware_utils/autoware_utils_uuid',
        'autoware_cmake/autoware_cmake',
        'auto_msgs/autoware_auto_planning_msgs',
        'auto_msgs/autoware_auto_geometry_msgs',
        'auto_msgs/autoware_auto_mapping_msgs',
    ]
    cleanup_core(CORE_DIR, KEEP_RELATIVE_CORE)

"""
Module for cleaning up the external/core directory.
"""
import shutil
from pathlib import Path
from typing import List

def cleanup_core(core_dir: Path, keep_relative: List[str]) -> None:
    """
    Remove all contents in core_dir except for the specified subdirectories.
    Args:
        core_dir: Path to the core directory.
        keep_relative: List of relative paths (from core_dir) to keep.
    """
    parent_relative = set(Path(rel).parts[0] for rel in keep_relative)
    keep_toplevel = set(parent_relative)
    keep_paths = [core_dir / rel for rel in keep_relative]

    for item in core_dir.iterdir():
        if item.name in keep_toplevel:
            continue
        print(f"Removing {item}")
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()

    for parent in keep_toplevel:
        parent_dir = core_dir / parent
        if not parent_dir.is_dir():
            continue
        for child in parent_dir.iterdir():
            if not any(child == kp for kp in keep_paths):
                print(f"Removing {child}")
                if child.is_dir():
                    shutil.rmtree(child)
                else:
                    child.unlink()

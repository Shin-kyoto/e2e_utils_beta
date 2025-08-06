#!/bin/sh

mkdir src/external
vcs import src/external < autoware.repos --shallow
python script/cleanup_external.py
cd src/external/vad;git apply ../../../.patches/0001-feat-autoware_tensorrt_vad-add-publisher-for-base_li.patch

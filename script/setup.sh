#!/bin/sh

mkdir src/external
vcs import src/external < autoware.repos --shallow
python script/cleanup_external.py

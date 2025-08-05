#!/bin/sh

mkdir external
vcs import external < autoware.repos --shallow
python script/cleanup_external.py

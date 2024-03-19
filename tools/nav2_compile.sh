#!/bin/bash
colcon build --cmake-clean-cache --symlink-install
ln -s src/bringup/fuzz.py

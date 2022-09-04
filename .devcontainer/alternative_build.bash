#!/bin/bash
build_type=$1; shift

source /home/bot/.bashrc
colcon build --mixin $build_type


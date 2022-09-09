#!/bin/bash

# Source our environment
source /home/bot/path_planner_ws/install/setup.bash

# Run gdb, forwarding all args through
/usr/bin/gdb ${@}

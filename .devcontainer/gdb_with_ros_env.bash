#!/bin/bash

# Source our environment
source /moveit2_app/moveit2_app/install/setup.bash

# Run gdb, forwarding all args through
/usr/bin/gdb ${@}

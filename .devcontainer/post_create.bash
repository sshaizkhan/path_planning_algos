#!/bin/bash

#update and upgrade any installed packages
sudo apt update
sudo apt -y upgrade

# Install terminator
sudo add-apt-repository ppa:gnome-terminator
sudo apt-get update
sudo apt-get install -y terminator

# Instll python-pip3
sudo apt-get -y install python3-pip

#install nano
sudo apt-get -y install nano

# Install bash-completion
sudo apt-get -y install bash-completion

# Install Clang-format-11
sudo apt-get -y install clang-format-11

# gnuplot
sudo apt-get install gnuplot

# install gdb
sudo apt install -y gdb
sudo apt-get -y install gdbserver

colcon --log-base "./colcon-log" mixin add default "file://`pwd`//path_planner_ws/src/path_planning_algos/.devcontainer/colcon_mixins/index.yaml"

sudo chown -R bot:bot /home/bot

ln -s /home/bot/path_planner_ws/src/path_planning_algos/.devcontainer/.vscode/ /home/bot/.vscode

# link the container settings
mkdir -p /home/bot/.vscode-server/data/Machine
ln -s /home/bot/path_planner_ws/src/path_planning_algos/.devcontainer/settings.json /home/bot/.vscode-server/data/Machine/settings.json

# install Terminator config
mkdir -p /home/bot/.config/terminator
ln -s /home/bot/path_planner_ws/src/path_planning_algos/.devcontainer/terminator.config /home/bot/.config/terminator/config

# link .bashrc
rm /home/bot/.bashrc
ln -s /home/bot/path_planner_ws/src/path_planning_algos/.devcontainer/.bashrc /home/bot/.bashrc

# link .bash_aliases
# rm /home/bot/.bash_aliases
ln -s /home/bot/path_planner_ws/src/path_planning_algos/.devcontainer/.bash_aliases /home/bot/.bash_aliases

#moveit dependencies
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y


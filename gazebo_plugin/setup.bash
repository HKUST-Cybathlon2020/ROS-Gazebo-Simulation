#! /bin/bash
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH::$(rospack find exoskeleton)/gazebo_plugin/build">>~/.bashrc

#!/usr/bin/env bash

source /apollo/ros_pkgs/devel/setup.bash
PATH=/usr/local/miniconda2/bin:$PATH roslaunch rosbridge_server rosbridge_websocket.launch
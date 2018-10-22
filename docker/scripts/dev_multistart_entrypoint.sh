#!/bin/bash

export PATH=${PATH}:/apollo/scripts:/usr/local/miniconda2/bin
source /apollo/scripts/apollo_base.sh

function on_exit {
  /apollo/scripts/bootstrap.sh stop
}
trap on_exit SIGINT SIGTERM

/apollo/scripts/bootstrap.sh start

if [ ! -d /apollo/lgsvl-rosbridge/ros_pkgs/devel ]; then
  pushd /apollo/lgsvl-rosbridge/ros_pkgs
  echo $PATH
  catkin_make
  popd
fi

/apollo/scripts/rosbridge.sh
on_exit

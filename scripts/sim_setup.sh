#!/usr/bin/env bash

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

source "${APOLLO_ROOT_DIR}/scripts/apollo_base.sh"

function start() {
  echo "Starting...."
  source "${APOLLO_ROOT_DIR}/ros_pkgs/devel/setup.bash"
  if [ $? -eq 0 ]; then
    PATH=/usr/local/miniconda2/bin:$PATH
    roslaunch rosbridge_server rosbridge_websocket.launch &
    rosrun simulator_image_converter simulator_image_converter &
  else
    echo "Failed to source ros_pkgs. Has it been built?"
  fi
}

function stop() {
  echo "Stopping...."
  pkill -SIGKILL -f "rosbridge"
  pkill -SIGKILL -f "image_converter"
  if [ $? -eq 0 ]; then
    echo "Successfully stopped."
  else
    echo "Not running - skipping."
  fi
}

case $1 in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    start
    ;;
esac

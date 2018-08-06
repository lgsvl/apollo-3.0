#!/usr/bin/env bash

# Script to (somewhat) gracefully stop all apollo related containers.

docker exec apollo_dev /apollo/scripts/bootstrap.sh stop

docker stop apollo_dev
docker stop apollo_yolo3d_volume
docker stop apollo_localization_volume
docker stop apollo_map_volume-sunnyvale_loop
docker stop apollo_map_volume-sunnyvale_big_loop

#!/usr/bin/env bash

# Script to (somewhat) gracefully stop all apollo related containers.

docker kill apollo_dev
docker kill apollo_yolo3d_volume
docker kill apollo_localization_volume
#docker kill apollo_map_volume-sunnyvale_loop
#docker kill apollo_map_volume-sunnyvale_big_loop

echo "Apollo docker containers have been killed."

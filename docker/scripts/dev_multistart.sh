#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

ARCH=$(uname -m)
VOLUME_VERSION="latest"
VERSION_OPT="nvidia"
VERSION=$VERSION_OPT
DOCKER_REPO=apolloauto/apollo

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

source ${APOLLO_ROOT_DIR}/scripts/apollo_base.sh

DEFAULT_MAPS=(
  sunnyvale_big_loop
  sunnyvale_loop
)
MAP_VOLUME_CONF=""

IMG="lgsvl/apollo"


function local_volumes() {
    # Apollo root and bazel cache dirs are required.
    volumes="-v $APOLLO_ROOT_DIR:/apollo \
             -v $HOME/.cache:${DOCKER_HOME}/.cache"
    case "$(uname -s)" in
        Linux)
            volumes="${volumes} -v /dev:/dev \
                                -v /media:/media \
                                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                                -v /etc/localtime:/etc/localtime:ro \
                                -v /usr/src:/usr/src \
                                -v /lib/modules:/lib/modules"
            ;;
        Darwin)
            # MacOS has strict limitations on mapping volumes.
            chmod -R a+wr ~/.cache/bazel
            ;;
    esac
    echo "${volumes}"
}

function main(){

    # Included default maps.
    for map_name in ${DEFAULT_MAPS[@]}; do
      MAP_VOLUME="apollo_map_volume-${map_name}"
      MAP_VOLUME_IMAGE=${DOCKER_REPO}:map_volume-${map_name}-${VOLUME_VERSION}
      if ! docker top ${MAP_VOLUME} &>/dev/null
      then
        docker run -t -d --rm --name ${MAP_VOLUME} ${MAP_VOLUME_IMAGE}
      fi
      MAP_VOLUME_CONF="${MAP_VOLUME_CONF} --volumes-from ${MAP_VOLUME}"
    done

    local display=""
    if [[ -z ${DISPLAY} ]];then
        display=":0"
    else
        display="${DISPLAY}"
    fi

    setup_device

    USER_ID=$(id -u)
    GRP=apollo
    GRP_ID=$(id -g)
    LOCAL_HOST=`hostname`
    DOCKER_HOME="/home/$USER"
    if [ "$USER" == "root" ];then
        DOCKER_HOME="/root"
    fi
    if [ ! -d "$HOME/.cache" ];then
        mkdir -p "$HOME/.cache"
    fi

    LOCALIZATION_VOLUME=apollo_localization_volume
    LOCALIZATION_VOLUME_IMAGE=${DOCKER_REPO}:localization_volume-${ARCH}-latest
    if ! docker top ${LOCALIZATION_VOLUME} &>/dev/null
    then
      docker run -t -d --rm --name ${LOCALIZATION_VOLUME} ${LOCALIZATION_VOLUME_IMAGE}
    fi

    YOLO3D_VOLUME=apollo_yolo3d_volume
    YOLO3D_VOLUME_IMAGE=${DOCKER_REPO}:yolo3d_volume-${ARCH}-latest
    if ! docker top ${YOLO3D_VOLUME} &>/dev/null
    then
      docker run -t -d --rm --name ${YOLO3D_VOLUME} ${YOLO3D_VOLUME_IMAGE}
    fi

    ROS_DOMAIN_ID=`python -c "import random; print(random.getrandbits(31))"`

    CONTAINER_NAME=apollo_dev$1

    docker network create ${CONTAINER_NAME}-net

    info "Starting docker container \"apollo_dev$1\" ..."
    docker run -ti \
	    --rm \
	    -d \
	    --runtime nvidia \
        --privileged \
        --name ${CONTAINER_NAME} \
        ${MAP_VOLUME_CONF} \
        --volumes-from ${LOCALIZATION_VOLUME} \
        --volumes-from ${YOLO3D_VOLUME} \
        -e DISPLAY=$display \
        -e DOCKER_USER=$USER \
        -e USER=$USER \
        -e DOCKER_USER_ID=$USER_ID \
        -e DOCKER_GRP="$GRP" \
        -e DOCKER_GRP_ID=$GRP_ID \
        -e DOCKER_IMG=$IMG \
        -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
        $(local_volumes) \
        --network ${CONTAINER_NAME}-net \
        -p 888$1:8888 \
        -p 909$1:9090 \
        -w /apollo \
        --add-host in_dev_docker:127.0.0.1 \
        --add-host ${LOCAL_HOST}:127.0.0.1 \
        --hostname in_dev_docker \
        --shm-size 2G \
        $IMG \
        /bin/bash

    info "Creating your user in \"${CONTAINER_NAME}\" ..."
    docker exec ${CONTAINER_NAME} bash -c "/apollo/scripts/docker_adduser.sh"

    info "Launching Apollo in \"${CONTAINER_NAME}\" ..."
    xhost +local:root 1>/dev/null 2>&1
    docker exec -u $USER -it ${CONTAINER_NAME} /apollo/docker/scripts/dev_multistart_entrypoint.sh
    xhost -local:root 1>/dev/null 2>&1

    info "Stopping docker container \"${CONTAINER_NAME}\" ..."
    docker kill ${CONTAINER_NAME}

    docker network rm ${CONTAINER_NAME}-net
}

main $*

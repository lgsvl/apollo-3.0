#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
# Modifications Copyright (c) 2018 LG Electronics, Inc. All Rights Reserved.
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


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${DIR}/.."

source "$DIR/apollo_base.sh"

function start() {
    eval "${APOLLO_BIN_PREFIX}/modules/planning/tools/planning_pad_terminal  \
        --log_dir=${APOLLO_ROOT_DIR}/data/log"
}

function stop() {
    pkill -SIGKILL -f planning_pad_terminal
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

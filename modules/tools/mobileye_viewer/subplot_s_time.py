#!/usr/bin/env python

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
import math

class SubplotSTime:
    def __init__(self, ax):
        self.s_speed_line, = ax.plot([0], [0], 'r-', lw=3, alpha=0.4)
        ax.set_xlim([-2, 100])
        ax.set_ylim([-0.5, 10])
        ax.set_xlabel("s (m)")
        ax.set_ylabel("time (sec)")


    def show(self, planning_data):
        planning_data.path_param_lock.acquire()
        self.s_speed_line.set_xdata(planning_data.s)
        self.s_speed_line.set_ydata(planning_data.relative_time)
        planning_data.path_param_lock.release()



/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 * Modifications Copyright (c) 2018 LG Electronics, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef MODULES_MONITOR_HARDWARE_RESOURCE_MONITOR_H_
#define MODULES_MONITOR_HARDWARE_RESOURCE_MONITOR_H_

#include <string>
#include <vector>

#include "modules/monitor/common/recurrent_runner.h"
#include "modules/monitor/proto/monitor_conf.pb.h"

namespace apollo {
namespace monitor {

class ResourceMonitor : public RecurrentRunner {
 public:
  explicit ResourceMonitor(const ResourceConf& config);
  void RunOnce(const double current_time) override;

 private:
  const ResourceConf& config_;
};

}  // namespace monitor
}  // namespace apollo

#endif  // MODULES_MONITOR_HARDWARE_RESOURCE_MONITOR_H_

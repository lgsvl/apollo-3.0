/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#define private public
#define protected public

#include "modules/planning/std_planning.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_context.h"
#include "modules/tools/fuzz/planning/proto/std_planning_fuzz.pb.h"
#include "libfuzzer/libfuzzer_macro.h"

static google::protobuf::LogSilencer logSilencer;

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;
using apollo::common::adapter::AdapterConfig;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::AdapterManagerConfig;
using apollo::tools::fuzz::planning::StdPlanningFuzzMessage;

class StdPlanningFuzz {
 public:
  void Init();
  void Fuzz(StdPlanningFuzzMessage std_planning_fuzz_message);

 private:
  std::unique_ptr<StdPlanning> std_planning_;
} std_planning_fuzzer;

void StdPlanningFuzz::Init() {
  AdapterManagerConfig config;
  config.set_is_ros(false);
  {
    auto *sub_config_1 = config.add_config();
    sub_config_1->set_type(AdapterConfig::LOCALIZATION);
    sub_config_1->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_1->set_message_history_limit(1);
    auto *sub_config_2 = config.add_config();
    sub_config_2->set_type(AdapterConfig::CHASSIS);
    sub_config_2->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_2->set_message_history_limit(1);
    auto *sub_config_3 = config.add_config();
    sub_config_3->set_type(AdapterConfig::ROUTING_RESPONSE);
    sub_config_3->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_3->set_message_history_limit(1);
    auto *sub_config_4 = config.add_config();
    sub_config_4->set_type(AdapterConfig::ROUTING_REQUEST);
    sub_config_4->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_4->set_message_history_limit(1);
    auto *sub_config_5 = config.add_config();
    sub_config_5->set_type(AdapterConfig::PREDICTION);
    sub_config_5->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_5->set_message_history_limit(10);
    auto *sub_config_6 = config.add_config();
    sub_config_6->set_type(AdapterConfig::PERCEPTION_OBSTACLES);
    sub_config_6->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_6->set_message_history_limit(1);
    auto *sub_config_7 = config.add_config();
    sub_config_7->set_type(AdapterConfig::PLANNING_TRAJECTORY);
    sub_config_7->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_7->set_message_history_limit(1);
    auto *sub_config_8 = config.add_config();
    sub_config_8->set_type(AdapterConfig::TRAFFIC_LIGHT_DETECTION);
    sub_config_8->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_8->set_message_history_limit(1);
    auto *sub_config_9 = config.add_config();
    sub_config_9->set_type(AdapterConfig::RELATIVE_MAP);
    sub_config_9->set_mode(AdapterConfig::RECEIVE_ONLY);
    sub_config_9->set_message_history_limit(1);
    auto *sub_config_10 = config.add_config();
    sub_config_10->set_type(AdapterConfig::MONITOR);
    sub_config_10->set_mode(AdapterConfig::PUBLISH_ONLY);
    sub_config_10->set_message_history_limit(1);
  }
  AdapterManager::Init(config);
  std_planning_.reset(new StdPlanning());
  std_planning_->Init();
}

void StdPlanningFuzz::Fuzz(StdPlanningFuzzMessage std_planning_fuzz_message) {
  AdapterManager::FeedLocalizationData(
      std_planning_fuzz_message.localization_estimate());
  AdapterManager::FeedChassisData(std_planning_fuzz_message.chassis());
  AdapterManager::FeedRoutingResponseData(
      std_planning_fuzz_message.routing_response());
  AdapterManager::FeedPredictionData(
      std_planning_fuzz_message.prediction_obstacles());
  std_planning_->RunOnce();
}

DEFINE_PROTO_FUZZER(const StdPlanningFuzzMessage& std_planning_fuzz_message) {
  std_planning_fuzzer.Fuzz(std_planning_fuzz_message);
}

extern "C" int LLVMFuzzerInitialize(int *argc, char ***argv) {
  std_planning_fuzzer.Init();
  return 0;
}

}  // namespace planning
}  // namespace apollo

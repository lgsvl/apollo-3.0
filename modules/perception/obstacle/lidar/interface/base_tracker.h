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
#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TRACKER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TRACKER_H_

// SAMPLE CODE:
//
// class MyTracker : public BaseTracker {
// public:
//     MyTracker() : BaseTracker() {}
//     virtual ~MyTracker() {}
//
//     virtual bool init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool track(
//              const std::vector<Object>& objects,
//              double timestamp,
//              const TrackerOptions& options,
//              std::vector<std::shared_ptr<Object>>* tracked_objects) override
//              {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string name() const override {
//          return "MyTracker";
//      }
//
// };
//
// // Register plugin.
// REGISTER_TRACKER(MyTracker);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseTracker* tracker =
//          BaseTrackerRegisterer::get_instance_by_name("MyTracker");
// using tracker to do somethings.
// ////////////////////////////////////////////////////

#include <memory>
#include <string>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/base/hdmap_struct.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace apollo {
namespace perception {

struct TrackerOptions {
  TrackerOptions() = default;
  explicit TrackerOptions(Eigen::Matrix4d *pose) : velodyne_trans(pose) {}

  std::shared_ptr<Eigen::Matrix4d> velodyne_trans;
  HdmapStructPtr hdmap = nullptr;
  HDMapInput *hdmap_input = NULL;
};

class BaseTracker {
 public:
  BaseTracker() {}
  virtual ~BaseTracker() {}

  virtual bool Init() = 0;

  // @brief: tracking objects.
  // @param [in]: current frame object list.
  // @param [in]: timestamp.
  // @param [in]: options.
  // @param [out]: current tracked objects.
  virtual bool Track(const std::vector<std::shared_ptr<Object>> &objects,
                     double timestamp, const TrackerOptions &options,
                     std::vector<std::shared_ptr<Object>> *tracked_objects) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTracker);
};

REGISTER_REGISTERER(BaseTracker);
#define REGISTER_TRACKER(name) REGISTER_CLASS(BaseTracker, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_INTERFACE_BASE_TRACKER_H_

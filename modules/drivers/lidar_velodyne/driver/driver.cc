/******************************************************************************
 * Modification Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017, Velodyne LiDAR INC., Algorithms and Signal Processing
 * Group
 *
 *  License: Modified BSD Software License Agreement
 *
 *  Id
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include "modules/drivers/lidar_velodyne/driver/driver.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <cmath>
#include <string>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "velodyne_msgs/VelodyneScan.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

namespace {
double time_offset = 0.0;
}

std::string toBinary(int n) {
  std::string r;
  while (n != 0) {
    r = (n % 2 == 0 ? "0" : "1") + r;
    n /= 2;
  }
  while (r.length() != 8) {
    r = '0' + r;
  }
  return r;
}

double convertBinaryToDecimal(std::string binaryString) {
  double value = 0;
  int indexCounter = 0;
  for (int i = binaryString.length() - 1; i >= 0; i--) {
    if (binaryString[i] == '1') {
      value += pow(2, indexCounter);
    }
    indexCounter++;
  }
  return value;
}

double computeTimeStamp(velodyne_msgs::VelodyneScanPtr scan, int index) {
  std::string digit4 = toBinary(scan->packets[index].data[1203]);
  std::string digit3 = toBinary(scan->packets[index].data[1202]);
  std::string digit2 = toBinary(scan->packets[index].data[1201]);
  std::string digit1 = toBinary(scan->packets[index].data[1200]);
  std::string digit =
      digit4 + digit3 + digit2 + digit1;  // string concatenation
  double value = convertBinaryToDecimal(digit);
  // compute the seconds from the beginning of that hour to when the data being
  // captured
  return value / 1000000.0;
}

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh) {
  // TODO(All): get proto from file here.
  // use private node handle to get parameters
  config_.frame_id = driver_node_conf_.frame_id();

  // get model name, validate string, determine packet rate
  config_.model = driver_node_conf_.model();
  double packet_rate = 0.0;  // packet frequency (Hz)
  std::string model_full_name;
  if ((config_.model == "64E_S2") || (config_.model == "64E_S2.1")) {
    // generates 1333312 points per second
    // 1 packet holds 384 points
    packet_rate = 3472.17;  // 1333312 / 384
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "64E") {
    packet_rate = 2600.0;
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "32E") {
    packet_rate = 1808.0;
    model_full_name = std::string("HDL-") + config_.model;
  } else if (config_.model == "VLP32C") {
    packet_rate = 1507;  // 12 groups of 32 firings where a pair of 2 firings
                         // corresponds to 55.296us -> 1/(12*55.296us)
    model_full_name = "VLP-32C";
  } else if (config_.model == "VLS128") {
    packet_rate = 1507;  // 3 groups of 128 firings where a set of 8 firings
                         // corresponds to 55.296us -> 1/(12*55.296us)
    model_full_name = "VLS-128";
  } else if (config_.model == "VLP16") {
    packet_rate = 754;  // 754 Packets/Second for Last or Strongest mode 1508
                        // for dual (VLP-16 User Manual)
    model_full_name = "VLP-16";
  } else {
    ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  config_.rpm = driver_node_conf_.rpm();

  AINFO << deviceName << " rotating at " << config_.rpm << " RPM";
  double frequency = (config_.rpm / 60.0);  // expected Hz rate
  AINFO << "publishing frequency: " << frequency;

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = driver_node_conf_.npackets();
  AINFO << "publishing " << config_.npackets << " packets per scan";

  std::string dump_file = driver_node_conf_.pcap();

  int udp_port = driver_node_conf_.port();

  // Initialize dynamic reconfigure
  // srv_ = boost::make_shared<
  //     dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig> >(
  //     private_nh);
  // dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::CallbackType
  //     f;
  // f = boost::bind(&VelodyneDriver::callback, this, _1, _2);
  // srv_->setCallback(f);  // Set callback function und call initially

  config_.time_offset = time_offset;

  // initialize diagnostics
  // diagnostics_.setHardwareID(deviceName);
  // const double diag_freq = packet_rate / config_.npackets;
  // diag_max_freq_ = diag_freq;
  // diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  // using namespace diagnostic_updater;
  // diag_topic_.reset(new TopicDiagnostic(
  //     "velodyne_packets", diagnostics_,
  //     FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
  //     TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "") {
    // have PCAP file?
    // read data from packet capture file
    input_.reset(new lidar_velodyne::InputPCAP(private_nh, udp_port,
                                               packet_rate, dump_file));
  } else {
    // read data from live socket
    input_.reset(new lidar_velodyne::InputSocket(private_nh, udp_port));
  }

  // raw packet output topic
  output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void) {
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  scan->packets.resize(config_.npackets);

  // Since the velodyne delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < config_.npackets; ++i) {
    while (true) {
      // keep reading until full packet received
      int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
      if (rc == 0) break;        // got a full packet?
      if (rc < 0) return false;  // end of file reached?
    }
  }
  // average the time stamp from first package and last package
  double firstTimeStamp = computeTimeStamp(scan, 0);
  double lastTimeStamp = computeTimeStamp(scan, config_.npackets - 1);
  double meanTimeStamp = (firstTimeStamp + lastTimeStamp) / 2;
  // std::cerr << " Velodyne Driver Timestamp first packet= " << firstTimeStamp
  // << std::endl;
  // std::cerr << " Velodyne Driver Timestamp last packet= " << lastTimeStamp <<
  // std::endl;
  time_t seconds;
  seconds = time(NULL);
  int gpsSeconds =
      (static_cast<int>(seconds / 3600)) * 3600 + floor(meanTimeStamp);
  int nanSecs = (meanTimeStamp - floor(meanTimeStamp)) * pow(10, 9);
  scan->header.stamp = ros::Time(gpsSeconds, nanSecs);
  // std::cerr<< scan->header.stamp << std::endl;
  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);
  // notify diagnostics that a message has been published, updating
  // its status
  // diag_topic_->tick(scan->header.stamp);
  // diagnostics_.update();

  return true;
}

// void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
//                               uint32_t level) {
//   ROS_INFO("Reconfigure Request");
//   config_.time_offset = config.time_offset;
// }

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

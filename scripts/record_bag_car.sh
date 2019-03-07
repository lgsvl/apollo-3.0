#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source "${DIR}/apollo_base.sh"

function start() {
  camera_short_topic="/apollo/sensor/camera/traffic/image_short "
  # camera_topics+="/apollo/sensor/camera/obstacle/front_6mm "
  camera_long_topic="/apollo/sensor/camera/traffic/image_long "

  lidar_unified_topic="/apollo/sensor/velodyne64/VelodyneScanUnified "
  lidar_raw_topic="/apollo/sensor/velodyne64/PointCloud2 "
  lidar_compensated_topic="/apollo/sensor/velodyne64/compensator/PointCloud2 "

  radar_topic="/apollo/sensor/conti_radar "

  gps_topics="/apollo/sensor/gnss/best_pose "
  gps_topics+="/apollo/sensor/gnss/corrected_imu "
  gps_topics+="/apollo/sensor/gnss/gnss_status "
  gps_topics+="/apollo/sensor/gnss/imu "
  gps_topics+="/apollo/sensor/gnss/raw_data "
  gps_topics+="/apollo/sensor/gnss/ins_stat "
  gps_topics+="/apollo/sensor/gnss/odometry "
  gps_topics+="/apollo/sensor/gnss/rtk_eph "
  gps_topics+="/apollo/sensor/gnss/rtk_obs "

  chassis_topics="/apollo/canbus/chassis "
  chassis_topics+="/apollo/canbus/chassis_detail "

  localization_topics="/apollo/localization/pose "
  perception_topics="/apollo/perception/obstacles "
  perception_topics+="/apollo/perception/traffic_light "
  prediction_topic="/apollo/prediction "

  record_cmd="rosbag record --split --size=2048 -b 2048 "
  record_cmd+="/apollo/drive_event "
  record_cmd+="/tf "
  record_cmd+="/tf_static "
  record_cmd+="/apollo/monitor "
  record_cmd+="/apollo/monitor/static_info "
  record_cmd+=$gps_topics
  record_cmd+=$chassis_topics
  record_cmd+=$radar_topic
  record_cmd+=$localization_topics
  record_cmd+=$perception_topics
  record_cmd+=$prediction_topic

  # parse arguments
  while getopts "hsliurcv" opt; do
    case $opt in
      h)
        help
        exit
        ;;
      u)
        echo "Recording VelodyneScanUnified topic."
        record_cmd+=$lidar_unified_topic
        ;;
      r)
        echo "Recording raw lidar topic."
        record_cmd+=$lidar_raw_topic
        ;;
      c)
        echo "Recording compensated lidar topic."
        record_cmd+=$lidar_compensated_topic
        ;;
      v)
        echo "Recording all lidar topics."
        record_cmd+=$lidar_raw_topic
        record_cmd+=$lidar_unified_topic
        record_cmd+=$lidar_compensated_topic
        ;;
      s)
        echo "Recording image_short topic."
        record_cmd+=$camera_short_topic
        ;;
      l)
        echo "Recording image_long topic."
        record_cmd+=$camera_long_topic
        ;;
      i)
        echo "Recording all image topics."
        record_cmd+=$camera_short_topic
        record_cmd+=$camera_long_topic
        ;;
    esac
  done

  if [ $# -eq 0 ]; then
    echo "Recording basic topics."
  fi

  # echo $record_cmd

  bag_dir="/apollo/data/bag/$(date '+%Y-%m-%d-%H-%M-%S')"
  mkdir $bag_dir
  cd $bag_dir

  # Start recording.
  record_bag_env_log

  NUM_PROCESSES="$(pgrep -c -f "rosbag record")"
  if [ "${NUM_PROCESSES}" -eq 0 ]; then
    $record_cmd
    echo $'\n'"Bag files written to $bag_dir"
  fi
}

function help() {
  echo "Usage:"
  echo
  echo "$0 -h                        Show this help message."
  echo "$0 -v                        Record all velodyne lidar topics."
  echo "$0 -i                        Record all image topics."
  echo
  echo "$0 -c                        Record /compensator/PointCloud2 topic"
  echo "$0 -r                        Record raw PointCloud2 topic"
  echo "$0 -u                        Record VelodyneScanUnified topic"
  echo "$0 -s                        Record /image_short topic"
  echo "$0 -l                        Record /image_long topic"
  echo
}

start $@

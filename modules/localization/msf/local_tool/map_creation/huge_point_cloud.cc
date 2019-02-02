#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "modules/common/log.h"
#include "modules/localization/msf/common/io/pcl_point_types.h"
#include "modules/localization/msf/common/io/velodyne_utility.h"

bool ParseCommandLine(int argc, char* argv[],
                      boost::program_options::variables_map* vm) {
  boost::program_options::options_description desc("Allowd options");
  desc.add_options()("help", "product help message")
      ("pcd_folders", boost::program_options::value<std::vector<std::string>>()
                          ->multitoken()
                          ->composing()
                          ->required(),
       "pcd folders(repeated)")(
          "pose_files",
          boost::program_options::value<std::vector<std::string>>()
              ->multitoken()
              ->composing()
              ->required(),
          "pose files(repeated)");
  try {
    boost::program_options::store(
        boost::program_options::parse_command_line(argc, argv, desc), *vm);
    if (vm->count("help")) {
      std::cerr << desc << std::endl;
      return false;
    }
    boost::program_options::notify(*vm);
  } catch (std::exception& e) {
    std::cerr << "Error" << e.what() << std::endl;
    std::cerr << desc << std::endl;
    return false;
  } catch (...) {
    std::cerr << "Unknown error!" << std::endl;
    return false;
  }
  return true;
}

int main(int argc, char** argv) {
  boost::program_options::variables_map boost_args;
  if (!ParseCommandLine(argc, argv, &boost_args)) {
    std::cerr << "Parse input command line failed." << std::endl;
    return -1;
  }

  const std::vector<std::string> pcd_folder_pathes =
      boost_args["pcd_folders"].as<std::vector<std::string>>();
  const std::vector<std::string> pose_files =
      boost_args["pose_files"].as<std::vector<std::string>>();
  if (pcd_folder_pathes.size() != pose_files.size()) {
    std::cerr << "The count of pcd folders is not equal pose files"
              << std::endl;
    return -1;
  }

  const unsigned int num_trials = pcd_folder_pathes.size();

  // load all poses
  std::cerr << "Pcd folders are as follows:" << std::endl;
  for (std::size_t i = 0; i < num_trials; ++i) {
    std::cerr << pcd_folder_pathes[i] << std::endl;
  }
  std::vector<std::vector<Eigen::Affine3d>> ieout_poses(num_trials);
  std::vector<std::vector<double>> time_stamps(num_trials);
  std::vector<std::vector<unsigned int>> pcd_indices(num_trials);
  for (std::size_t i = 0; i < pose_files.size(); ++i) {
    apollo::localization::msf::velodyne::LoadPcdPoses(
        pose_files[i], &ieout_poses[i], &time_stamps[i], &pcd_indices[i]);
  }

  Eigen::Affine3d initial_pose_inv;

  pcl::PointCloud<apollo::localization::msf::velodyne::PointXYZIT>::Ptr result_cloud(new pcl::PointCloud<apollo::localization::msf::velodyne::PointXYZIT>);

  for (unsigned int trial = 0; trial < num_trials; ++trial) {
    for (unsigned int frame_idx = 0; frame_idx < ieout_poses[trial].size();
         ++frame_idx) {
      unsigned int trial_frame_idx = frame_idx;
      const std::vector<Eigen::Affine3d>& poses = ieout_poses[trial];

      apollo::localization::msf::velodyne::VelodyneFrame velodyne_frame;
      std::string pcd_file_path;
      std::ostringstream ss;
      ss << pcd_indices[trial][frame_idx];
      pcd_file_path = pcd_folder_pathes[trial] + "/" + ss.str() + ".pcd";
      const Eigen::Affine3d& pcd_pose = poses[trial_frame_idx];

      if (trial_frame_idx == 0) {
          initial_pose_inv = pcd_pose.inverse();
      }

      pcl::PointCloud<apollo::localization::msf::velodyne::PointXYZIT>::Ptr cloud(new pcl::PointCloud<apollo::localization::msf::velodyne::PointXYZIT>);
      if (pcl::io::loadPCDFile(pcd_file_path, *cloud) >= 0) {
        pcl::transformPointCloud (*cloud, *cloud, pcd_pose);
        pcl::transformPointCloud (*cloud, *cloud, initial_pose_inv);

        *result_cloud+=*cloud;

      } else {
        AERROR << "Failed to load PCD file: " << pcd_file_path;
      }

      pcl::io::savePCDFile("result_pcd.pcd", *result_cloud);
      AERROR << "File " << frame_idx << " out of " << ieout_poses[trial].size();

    }
  }

  return 0;
}

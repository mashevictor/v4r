/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file  segmenter.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date   April, 2016
 * @brief Base class for segmentation
 *
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/core/macros.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS SegmenterParameter {
 public:
  size_t min_cluster_size_;    ///< minimum number of points in a cluster
  size_t max_cluster_size_;    ///< minimum number of points in a cluster
  double distance_threshold_;  ///< tolerance in meters for difference in perpendicular distance (d component of plane
                               /// equation) to the plane between neighboring points, to be considered part of the same
  /// plane
  double angular_threshold_deg_;  ///< tolerance in gradients for difference in normal direction between neighboring
                                  /// points, to be considered part of the same plane
  int wsize_;
  float cluster_tolerance_;
  SegmenterParameter(size_t min_cluster_size = 500, size_t max_cluster_size = std::numeric_limits<int>::max(),
                     double distance_threshold = 0.01f,  // 0.035f
                     double angular_threshold_deg = 10.f, int wsize = 5, float cluster_tolerance = 0.05f)
  : min_cluster_size_(min_cluster_size), max_cluster_size_(max_cluster_size), distance_threshold_(distance_threshold),
    angular_threshold_deg_(angular_threshold_deg), wsize_(wsize), cluster_tolerance_(cluster_tolerance) {}

  virtual ~SegmenterParameter() {}

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  std::vector<std::string> init(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    return init(arguments);
  }

  /**
   * @brief init parameters
   * @param command_line_arguments (according to Boost program options library)
   * @return unused parameters (given parameters that were not used in this initialization call)
   */
  std::vector<std::string> init(const std::vector<std::string> &command_line_arguments) {
    po::options_description desc("Segmentation Parameter\n=====================\n");
    desc.add_options()("help,h", "produce help message")(
        "seg_min_cluster_size", po::value<size_t>(&min_cluster_size_)->default_value(min_cluster_size_),
        "minimum number of points in a cluster")(
        "seg_max_cluster_size", po::value<size_t>(&max_cluster_size_)->default_value(max_cluster_size_), "")(
        "seg_distance_threshold", po::value<double>(&distance_threshold_)->default_value(distance_threshold_),
        "tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane "
        "between neighboring points, to be considered part of the same plane")(
        "seg_angular_threshold_deg", po::value<double>(&angular_threshold_deg_)->default_value(angular_threshold_deg_),
        "tolerance in gradients for difference in normal direction between neighboring points, to be considered part "
        "of the same plane.")("seg_wsize", po::value<int>(&wsize_)->default_value(wsize_), "")(
        "seg_object_cluster_tolerance", po::value<float>(&cluster_tolerance_)->default_value(cluster_tolerance_), "");
    po::variables_map vm;
    po::parsed_options parsed =
        po::command_line_parser(command_line_arguments).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      to_pass_further.push_back("-h");
    }
    try {
      po::notify(vm);
    } catch (std::exception &e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    }
    return to_pass_further;
  }
};

template <typename PointT>
class V4R_EXPORTS Segmenter {
 protected:
  typename pcl::PointCloud<PointT>::ConstPtr scene_;  ///< point cloud to be segmented
  pcl::PointCloud<pcl::Normal>::ConstPtr normals_;    ///< normals of the cloud to be segmented
  std::vector<std::vector<int>>
      clusters_;  ///< segmented clusters. Each cluster represents a bunch of indices of the input cloud

 public:
  virtual ~Segmenter() {}

  Segmenter() {}

  /**
   * @brief sets the cloud which ought to be segmented
   * @param cloud
   */
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud) {
    scene_ = cloud;
  }

  /**
   * @brief sets the normals of the cloud which ought to be segmented
   * @param normals
   */
  void setNormalsCloud(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals) {
    normals_ = normals;
  }

  /**
   * @brief get segmented indices
   * @param indices
   */
  void getSegmentIndices(std::vector<std::vector<int>> &indices) const {
    indices = clusters_;
  }

  virtual bool getRequiresNormals() = 0;

  /**
   * @brief segment
   */
  virtual void segment() = 0;

  typedef boost::shared_ptr<Segmenter<PointT>> Ptr;
  typedef boost::shared_ptr<Segmenter<PointT> const> ConstPtr;
};
}

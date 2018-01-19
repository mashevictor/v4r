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
 * @file   types.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date   April, 2016
 * @brief  smooth Euclidean segmentation
 *
 */

#pragma once

#include <pcl/octree/octree.h>
#include <v4r/core/macros.h>
#include <v4r/segmentation/segmenter.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r {

class SmoothEuclideanSegmenterParameter : public SegmenterParameter {
 public:
  float eps_angle_threshold_deg_;
  float curvature_threshold_;
  float cluster_tolerance_;
  size_t min_points_;
  bool z_adaptive_;  ///< if true, scales the smooth segmentation parameters linear with distance (constant till 1m at
                     /// the given parameters)
  float octree_resolution_;
  bool force_unorganized_;  ///< if true, searches for neighboring points using the search tree and not pixel neighbors
                            ///(even though input cloud is organized)
  bool compute_planar_patches_only_;  ///< if true, only compute planar surface patches
  float planar_inlier_dist_;          ///< maximum allowed distance of a point to the plane

  SmoothEuclideanSegmenterParameter(float eps_angle_threshold_deg = 5.f,  // 0.25f
                                    float curvature_threshold = 0.04f,
                                    float cluster_tolerance = 0.01f,  // 0.015f;
                                    size_t min_points = 100,          // 20
                                    bool z_adaptive = true, float octree_resolution = 0.01f,
                                    bool force_unorganized = false, bool compute_planar_patches_only = false,
                                    float planaer_inlier_dist = 0.02f)
  : eps_angle_threshold_deg_(eps_angle_threshold_deg), curvature_threshold_(curvature_threshold),
    cluster_tolerance_(cluster_tolerance), min_points_(min_points), z_adaptive_(z_adaptive),
    octree_resolution_(octree_resolution), force_unorganized_(force_unorganized),
    compute_planar_patches_only_(compute_planar_patches_only), planar_inlier_dist_(planaer_inlier_dist) {}

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
    po::options_description desc("Smooth Region Growing Segmentation Parameters\n=====================");
    desc.add_options()("help,h", "produce help message")(
        "min_cluster_size", po::value<size_t>(&min_points_)->default_value(min_points_), "")(
        "sensor_noise_max", po::value<float>(&cluster_tolerance_)->default_value(cluster_tolerance_), "")
        //                ("chop_z_segmentation", po::value<double>(&chop_z_)->default_value(chop_z_), "")
        ("eps_angle_threshold", po::value<float>(&eps_angle_threshold_deg_)->default_value(eps_angle_threshold_deg_),
         "smooth clustering parameter for the angle threshold")(
            "curvature_threshold", po::value<float>(&curvature_threshold_)->default_value(curvature_threshold_),
            "smooth clustering parameter for curvate");
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
class V4R_EXPORTS SmoothEuclideanSegmenter : public Segmenter<PointT> {
  using Segmenter<PointT>::normals_;
  using Segmenter<PointT>::clusters_;
  using Segmenter<PointT>::scene_;

  typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr octree_;
  SmoothEuclideanSegmenterParameter param_;

 public:
  SmoothEuclideanSegmenter(const SmoothEuclideanSegmenterParameter &p = SmoothEuclideanSegmenterParameter())
  : param_(p) {}

  bool getRequiresNormals() {
    return true;
  }

  void setSearchMethod(const typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr &octree) {
    octree_ = octree;
  }

  void segment();

  typedef boost::shared_ptr<SmoothEuclideanSegmenter<PointT>> Ptr;
  typedef boost::shared_ptr<SmoothEuclideanSegmenter<PointT> const> ConstPtr;
};
}

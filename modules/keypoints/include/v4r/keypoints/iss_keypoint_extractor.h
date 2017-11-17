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
 * @file iss_keypoint_extractor.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#pragma once

#include <v4r/keypoints/keypoint_extractor.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r {

class V4R_EXPORTS IssKeypointExtractorParameter  // see PCL documentation for further details
{
 public:
  double salient_radius_;  ///< Set the radius of the spherical neighborhood used to compute the scatter matrix.
  double non_max_radius_;  ///< Set the radius for the application of the non maxima supression algorithm.
  double normal_radius_;   ///< Set the radius used for the estimation of the surface normals of the input cloud. If the
                           /// radius is too large, the temporal performances of the detector may degrade significantly.
  /// Only used if parameter with_border_estimation equal true.
  double border_radius_;  ///< Set the radius used for the estimation of the boundary points. If the radius is too
                          /// large, the temporal performances of the detector may degrade significantly. Only used if
  /// parameter with_border_estimation equal true.
  double gamma_21_;    ///< Set the upper bound on the ratio between the second and the first eigenvalue.
  double gamma_32_;    ///< Set the upper bound on the ratio between the third and the second eigenvalue.
  int min_neighbors_;  ///< Set the minimum number of neighbors that has to be found while applying the non maxima
                       /// suppression algorithm.
  bool with_border_estimation_;
  int threads_;
  float angle_thresh_deg_;

  IssKeypointExtractorParameter()
  : salient_radius_(0.02f),                                                                               //(6*0.005f),
    non_max_radius_(4 * 0.005f), normal_radius_(4 * 0.005f), border_radius_(1 * 0.005f), gamma_21_(0.8),  //(0.975),
    gamma_32_(0.8),                                                                                       //(0.975),
    min_neighbors_(5), with_border_estimation_(false), threads_(4), angle_thresh_deg_(60.f) {}

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
    po::options_description desc("ISS Keypoint Extractor Parameter\n=====================\n");
    desc.add_options()("help,h", "produce help message")(
        "iss_salient_radius", po::value<double>(&salient_radius_)->default_value(salient_radius_),
        "Set the radius of the spherical neighborhood used to compute the scatter matrix.")(
        "iss_non_max_radius", po::value<double>(&non_max_radius_)->default_value(non_max_radius_),
        "Set the radius for the application of the non maxima supression algorithm.")(
        "iss_normal_radius", po::value<double>(&normal_radius_)->default_value(normal_radius_),
        " Set the radius used for the estimation of the surface normals of the input cloud. If the radius is too "
        "large, the temporal performances of the detector may degrade significantly. Only used if parameter "
        "with_border_estimation equal true.")(
        "iss_border_radius", po::value<double>(&border_radius_)->default_value(border_radius_),
        "Set the radius used for the estimation of the boundary points. If the radius is too large, the temporal "
        "performances of the detector may degrade significantly. Only used if parameter with_border_estimation equal "
        "true.")("iss_gamma_21", po::value<double>(&gamma_21_)->default_value(gamma_21_),
                 "Set the upper bound on the ratio between the second and the first eigenvalue")(
        "iss_gamma_32", po::value<double>(&gamma_32_)->default_value(gamma_32_),
        "Set the upper bound on the ratio between the third and the second eigenvalue.")(
        "iss_min_neighbors", po::value<int>(&min_neighbors_)->default_value(min_neighbors_),
        "Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm")(
        "iss_with_border_estimation", po::value<bool>(&with_border_estimation_)->default_value(with_border_estimation_),
        "")("iss_threads", po::value<int>(&threads_)->default_value(threads_), "number of threads")(
        "iss_angle_thresh_deg", po::value<float>(&angle_thresh_deg_)->default_value(angle_thresh_deg_),
        "Set the decision boundary (angle threshold) that marks points as boundary or regular.");
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
class V4R_EXPORTS IssKeypointExtractor : public KeypointExtractor<PointT> {
 private:
  typedef typename pcl::PointCloud<PointT>::Ptr PointInTPtr;
  using KeypointExtractor<PointT>::input_;
  using KeypointExtractor<PointT>::normals_;
  using KeypointExtractor<PointT>::indices_;
  using KeypointExtractor<PointT>::keypoints_;
  using KeypointExtractor<PointT>::keypoint_indices_;

  IssKeypointExtractorParameter param_;

 public:
  IssKeypointExtractor(const IssKeypointExtractorParameter &p = IssKeypointExtractorParameter()) : param_(p) {}

  void compute();

  bool needNormals() const {
    return true;
  }

  int getKeypointExtractorType() const {
    return KeypointType::ISS;
  }

  std::string getKeypointExtractorName() const {
    return "iss";
  }

  typedef boost::shared_ptr<IssKeypointExtractor<PointT>> Ptr;
  typedef boost::shared_ptr<IssKeypointExtractor<PointT> const> ConstPtr;
};
}

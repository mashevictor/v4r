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
 * @file global_alexnet_cnn_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */
#pragma once

#include <opencv/cv.h>
#include <v4r/features/global_estimator.h>

#include <glog/logging.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace caffe {
template <typename Dtype>
class Net;
}

namespace v4r {

class V4R_EXPORTS CNN_Feat_ExtractorParameter {
 public:
  size_t image_height_;
  size_t image_width_;
  int device_id_;
  std::string device_name_;
  std::string output_layer_name_;  ///< name of the layer of the CNN that is used for feature extraction
  std::string feature_extraction_proto_, pretrained_binary_proto_, input_mean_file_;
  cv::Vec3b background_color_;  ///< background color of pixels not representing the object (only used if remove
                                /// background is enabled)
  bool remove_background_;      ///< if true, removes background for pixels not representing the object
  int margin_;                  ///< margin in pixel added to each side of the object's bounding box
  CNN_Feat_ExtractorParameter()
  : image_height_(256), image_width_(256), device_id_(0), device_name_("CPU"), output_layer_name_("fc7"),
    background_color_(cv::Vec3b(255, 255, 255)), remove_background_(false), margin_(10) {}

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
    po::options_description desc("CNN parameters\n=====================");
    desc.add_options()("help,h", "produce help message")(
        "cnn_net", po::value<std::string>(&feature_extraction_proto_)->required(), "Definition of CNN (.prototxt)")(
        "cnn_pretrained_net", po::value<std::string>(&pretrained_binary_proto_)->required(),
        "Trained weights (.caffemodel)")("cnn_input_mean_file", po::value<std::string>(&input_mean_file_)->required(),
                                         "mean pixel values (.binaryproto)")(
        "cnn_device_name", po::value<std::string>(&device_name_)->default_value(device_name_), "")(
        "cnn_output_layer_name", po::value<std::string>(&output_layer_name_)->default_value(output_layer_name_), "")(
        "cnn_device_id", po::value<int>(&device_id_)->default_value(device_id_), "")(
        "cnn_image_height", po::value<size_t>(&image_height_)->default_value(image_height_), "")(
        "cnn_remove_background", po::value<bool>(&remove_background_)->default_value(remove_background_),
        "if true, removes background for pixels not representing the object")(
        "cnn_margin", po::value<int>(&margin_)->default_value(margin_),
        "margin in pixel added to each side of the object's bounding box");
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

/**
 * @brief Feature extraction from a Convolutional Neural Network based on Berkeley's Caffe Framework.
 * Extracts the image from a RGB-D point cloud by the bounding box indicated from the object indices
 * @author Thomas Faeulhammer
 * @date Nov, 2015
 */
template <typename PointT, typename Dtype = float>
class V4R_EXPORTS CNN_Feat_Extractor : public GlobalEstimator<PointT> {
 public:
  using GlobalEstimator<PointT>::indices_;
  using GlobalEstimator<PointT>::cloud_;
  using GlobalEstimator<PointT>::descr_name_;
  using GlobalEstimator<PointT>::descr_type_;
  using GlobalEstimator<PointT>::feature_dimensions_;

  CNN_Feat_ExtractorParameter param_;

 private:
  boost::shared_ptr<caffe::Net<Dtype>> net_;
  bool init_;
  cv::Mat mean_;
  cv::Size input_geometry_;
  int num_channels_;

  void SetMean(const std::string &mean_file);
  void Preprocess(const cv::Mat &img, std::vector<cv::Mat> *input_channels);
  void WrapInputLayer(std::vector<cv::Mat> *input_channels);
  int init();

 public:
  CNN_Feat_Extractor(const CNN_Feat_ExtractorParameter &p = CNN_Feat_ExtractorParameter(),
                     const std::string &descr_name = "alexnet", size_t descr_type = FeatureType::ALEXNET,
                     size_t feature_dimensions = 4096)
  : GlobalEstimator<PointT>(descr_name, descr_type, feature_dimensions), param_(p), init_(false) {}

  bool compute(Eigen::MatrixXf &signature);

  bool compute(const cv::Mat &img, Eigen::MatrixXf &signature);

  bool needNormals() const {
    return false;
  }

  typedef boost::shared_ptr<CNN_Feat_Extractor<PointT, Dtype>> Ptr;
  typedef boost::shared_ptr<CNN_Feat_Extractor<PointT, Dtype> const> ConstPtr;
};
}

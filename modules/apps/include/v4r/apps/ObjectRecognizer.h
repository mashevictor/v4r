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
 * @file ObjectRecognizer.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/apps/CloudSegmenter.h>
#include <v4r/apps/ObjectRecognizerParameter.h>
#include <v4r/apps/visualization.h>
#include <v4r/common/normals.h>
#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/recognition/local_recognition_pipeline.h>
#include <v4r/recognition/multi_pipeline_recognizer.h>
#include <boost/serialization/vector.hpp>

namespace bf = boost::filesystem;

namespace v4r {

namespace apps {

template <typename PointT>
class V4R_EXPORTS ObjectRecognizer {
 private:
  typename v4r::RecognitionPipeline<PointT>::Ptr mrec_;                             ///< multi-pipeline recognizer
  typename v4r::LocalRecognitionPipeline<PointT>::Ptr local_recognition_pipeline_;  ///< local recognition pipeline
                                                                                    ///(member variable just because of
                                                                                    ///visualization of keypoints)
  typename v4r::HypothesisVerification<PointT, PointT>::Ptr hv_;                    ///< hypothesis verification object
  typename v4r::NormalEstimator<PointT>::Ptr
      normal_estimator_;  ///< normal estimator used for computing surface normals (currently only used at training)

  typename v4r::ObjectRecognitionVisualizer<PointT>::Ptr rec_vis_;  ///< visualization object

  typename v4r::apps::CloudSegmenter<PointT>::Ptr cloud_segmenter_;  ///< cloud segmenter for plane removal (if enabled)

  bool visualize_;          ///< if true, visualizes objects
  bool skip_verification_;  ///< if true, will only generate hypotheses but not verify them
  bf::path models_dir_;

  ObjectRecognizerParameter param_;

  Camera::ConstPtr camera_;

  typename Source<PointT>::Ptr model_database_;

  // MULTI-VIEW STUFF
  class View {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    typename pcl::PointCloud<PointT>::Ptr processed_cloud_;
    typename pcl::PointCloud<PointT>::Ptr removed_points_;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
    std::vector<std::vector<float>> pt_properties_;
    Eigen::Matrix4f camera_pose_;
  };
  std::vector<View> views_;  ///< all views in sequence

  /**
   * @brief detectChanges detect changes in multi-view sequence (e.g. objects removed or added to the scene within
   * observation period)
   * @param v current view
   */
  void detectChanges(View &v);

  typename pcl::PointCloud<PointT>::Ptr registered_scene_cloud_;  ///< registered point cloud of all processed input
                                                                  ///clouds in common camera reference frame

  std::vector<std::pair<std::string, float>>
      elapsed_time_;  ///< measurements of computation times for various components

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ObjectRecognizer() : visualize_(false), skip_verification_(false) {}

  /**
   * @brief initialize initialize Object recognizer (sets up model database, recognition pipeline and hypotheses
   * verification)
   * @param argc
   * @param argv
   */
  void initialize(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    initialize(arguments);
  }

  /**
   * @brief initialize initialize Object recognizer (sets up model database, recognition pipeline and hypotheses
   * verification)
   * @param arguments
   */
  void initialize(std::vector<std::string> &command_line_arguments,
                  const boost::filesystem::path &config_folder = bf::path("cfg"));

  /**
   * @brief recognize recognize objects in point cloud
   * @param cloud (organized) point cloud
   * @return
   */
  std::vector<ObjectHypothesesGroup> recognize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

  typename pcl::PointCloud<PointT>::ConstPtr getModel(const std::string &model_name, int resolution_mm) const {
    bool found;
    typename Source<PointT>::ConstPtr mdb = mrec_->getModelDatabase();
    typename Model<PointT>::ConstPtr model = mdb->getModelById("", model_name, found);
    if (!found) {
      std::cerr << "Could not find model with name " << model_name << std::endl;
      typename pcl::PointCloud<PointT>::ConstPtr foo;
      return foo;
    }

    return model->getAssembled(resolution_mm);
  }

  bf::path getModelsDir() const {
    return models_dir_;
  }

  void setModelsDir(const bf::path &dir) {
    models_dir_ = dir;
  }

  /**
   * @brief getElapsedTimes
   * @return compuation time measurements for various components
   */
  std::vector<std::pair<std::string, float>> getElapsedTimes() const {
    return elapsed_time_;
  }

  /**
   * @brief getParam get recognition parameter
   * @return parameter
   */
  ObjectRecognizerParameter getParam() const {
    return param_;
  }

  /**
   * @brief getCamera get pointer to camera parameters
   * @return camera parameter
   */
  Camera::ConstPtr getCamera() const {
    return camera_;
  }

  /**
   * @brief setCamera set the camera used for z-buffering
   * @param cam camera parameters
   */
  void setCamera(const Camera::ConstPtr &cam) {
    camera_ = cam;

    if (hv_) {
      hv_->setCamera(camera_);
    }
  }

  /**
   * @brief resetMultiView resets all state variables of the multi-view and initializes a new multi-view sequence
   */
  void resetMultiView();
};
}
}

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
 * @file global_recognition_pipeline.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <omp.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/recognition/global_recognizer.h>
#include <v4r/recognition/recognition_pipeline.h>
#include <v4r/segmentation/all_headers.h>

namespace v4r {
/**
 * @brief This class merges the results of various global descriptors into a set of hypotheses.
 * @author Thomas Faeulhammer
 * @date Jan 2017
 */
template <typename PointT>
class V4R_EXPORTS GlobalRecognitionPipeline : public RecognitionPipeline<PointT> {
 private:
  using RecognitionPipeline<PointT>::elapsed_time_;
  using RecognitionPipeline<PointT>::scene_;
  using RecognitionPipeline<PointT>::scene_normals_;
  using RecognitionPipeline<PointT>::obj_hypotheses_;
  using RecognitionPipeline<PointT>::normal_estimator_;
  using RecognitionPipeline<PointT>::m_db_;
  using RecognitionPipeline<PointT>::table_plane_;
  using RecognitionPipeline<PointT>::table_plane_set_;
  using RecognitionPipeline<PointT>::vis_param_;

  bool visualize_clusters_;  ///< If set, visualizes the cluster and displays recognition information for each
  mutable std::vector<std::string> coordinate_axis_ids_global_;

  //    omp_lock_t rec_lock_;

  typename Segmenter<PointT>::Ptr seg_;
  typename PlaneExtractor<PointT>::Ptr plane_extractor_;
  std::vector<std::vector<int>> clusters_;
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> planes_;  ///< extracted planes

  std::vector<typename GlobalRecognizer<PointT>::Ptr>
      global_recognizers_;  ///< set of Global recognizer generating keypoint correspondences
  std::vector<ObjectHypothesesGroup>
      obj_hypotheses_wo_elongation_check_;  ///< just for visualization (to see effect of elongation check)

  void visualize();

  /**
   * @brief recognize
   */
  void do_recognize();

  void doInit(const bf::path &trained_dir, bool force_retrain,
              const std::vector<std::string> &object_instances_to_load);

 public:
  GlobalRecognitionPipeline() : visualize_clusters_(false) {}

  /**
   * @brief addRecognizer
   * @param rec Global recognizer
   */
  void addRecognizer(const typename GlobalRecognizer<PointT>::Ptr &l_rec) {
    global_recognizers_.push_back(l_rec);
  }

  /**
   * @brief needNormals
   * @return
   */
  bool needNormals() const {
    for (size_t r_id = 0; r_id < global_recognizers_.size(); r_id++) {
      if (global_recognizers_[r_id]->needNormals())
        return true;
    }

    return false;
  }

  /**
   * @brief setSegmentationAlgorithm
   * @param seg
   */
  void setSegmentationAlgorithm(const typename Segmenter<PointT>::Ptr &seg) {
    seg_ = seg;
  }

  /**
   * @brief getFeatureType
   * @return
   */
  size_t getFeatureType() const {
    size_t feat_type = 0;
    for (size_t r_id = 0; r_id < global_recognizers_.size(); r_id++)
      feat_type += global_recognizers_[r_id]->getFeatureType();

    return feat_type;
  }

  /**
   * @brief requiresSegmentation
   * @return
   */
  bool requiresSegmentation() const {
    return true;
  }

  /**
   * @brief setVisualizeClusters
   * @param visualize if true, will visualize segmented clusters and the object classified for each of them
   */
  void setVisualizeClusters(bool visualize = true) {
    visualize_clusters_ = visualize;
  }

  typedef boost::shared_ptr<GlobalRecognitionPipeline<PointT>> Ptr;
  typedef boost::shared_ptr<GlobalRecognitionPipeline<PointT> const> ConstPtr;
};
}

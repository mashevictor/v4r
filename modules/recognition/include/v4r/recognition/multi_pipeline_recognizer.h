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
 * @file multi_pipeline_recognizer.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#pragma once

#include <omp.h>
#include <v4r/recognition/recognition_pipeline.h>

namespace v4r {
template <typename PointT>
class V4R_EXPORTS MultiRecognitionPipeline : public RecognitionPipeline<PointT> {
 private:
  using RecognitionPipeline<PointT>::elapsed_time_;
  using RecognitionPipeline<PointT>::scene_;
  using RecognitionPipeline<PointT>::scene_normals_;
  using RecognitionPipeline<PointT>::m_db_;
  using RecognitionPipeline<PointT>::normal_estimator_;
  using RecognitionPipeline<PointT>::obj_hypotheses_;
  using RecognitionPipeline<PointT>::table_plane_;
  using RecognitionPipeline<PointT>::table_plane_set_;
  using RecognitionPipeline<PointT>::vis_param_;

  std::vector<typename RecognitionPipeline<PointT>::Ptr> recognition_pipelines_;

  omp_lock_t rec_lock_;

  /**
   * @brief recognize
   */
  void do_recognize();

 public:
  MultiRecognitionPipeline() {}

  /**
       * @brief initialize the recognizer (extract features, create FLANN,...)
       * @param[in] path to model database. If training directory exists, will load trained model from disk; if not,
   * computed features will be stored on disk (in each
       * object model folder, a feature folder is created with data)
       * @param[in] retrain if set, will re-compute features and store to disk, no matter if they already exist or not
       */
  void initialize(const bf::path &trained_dir = "", bool retrain = false);

  /**
   * @brief oh_tmp
   * @param rec recognition pipeline (local or global)
   */
  void addRecognitionPipeline(typename RecognitionPipeline<PointT>::Ptr &rec) {
    recognition_pipelines_.push_back(rec);
  }

  /**
       * @brief needNormals
       * @return true if normals are needed, false otherwise
       */
  bool needNormals() const {
    for (size_t r_id = 0; r_id < recognition_pipelines_.size(); r_id++) {
      if (recognition_pipelines_[r_id]->needNormals())
        return true;
    }
    return false;
  }

  /**
   * @brief getFeatureType
   * @return
   */
  size_t getFeatureType() const {
    size_t feat_type = 0;
    for (size_t r_id = 0; r_id < recognition_pipelines_.size(); r_id++)
      feat_type += recognition_pipelines_[r_id]->getFeatureType();

    return feat_type;
  }

  /**
   * @brief requiresSegmentation
   * @return
   */
  bool requiresSegmentation() const {
    bool ret_value = false;
    for (size_t i = 0; (i < recognition_pipelines_.size()) && !ret_value; i++)
      ret_value = recognition_pipelines_[i]->requiresSegmentation();

    return ret_value;
  }

  std::vector<typename RecognitionPipeline<PointT>::Ptr> getRecognitionPipelines() const {
    return recognition_pipelines_;
  }

  typedef boost::shared_ptr<MultiRecognitionPipeline<PointT>> Ptr;
  typedef boost::shared_ptr<MultiRecognitionPipeline<PointT> const> ConstPtr;
};
}

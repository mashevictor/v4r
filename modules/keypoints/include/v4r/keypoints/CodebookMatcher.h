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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#ifndef V4R_CODEBOOK_MATCHER_HH
#define V4R_CODEBOOK_MATCHER_HH

#include <float.h>
#include <v4r/common/ClusteringRNN.h>
#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include <vector>

namespace v4r {

class V4R_EXPORTS CodebookMatcher {
 public:
  class V4R_EXPORTS Parameter {
   public:
    float thr_desc_rnn;
    float nnr;
    float max_dist;
    Parameter(float _thr_desc_rnn = 0.55, float _nnr = 0.92, float _max_dist = .7)
    : thr_desc_rnn(_thr_desc_rnn), nnr(_nnr), max_dist(_max_dist) {}
  };

 private:
  Parameter param;

  ClusteringRNN rnn;

  int max_view_index;
  DataMatrix2Df descs;
  std::vector<std::pair<int, int>> vk_indices;

  cv::Mat cb_centers;
  std::vector<std::vector<std::pair<int, int>>> cb_entries;
  std::vector<std::pair<int, int>> view_rank;

  cv::Ptr<cv::DescriptorMatcher> matcher;

 public:
  cv::Mat dbg;

  CodebookMatcher(const Parameter &p = Parameter());
  ~CodebookMatcher();

  void clear();
  void addView(const cv::Mat &descriptors, int view_idx);
  void createCodebook();
  void createCodebook(cv::Mat &_cb_centers, std::vector<std::vector<std::pair<int, int>>> &_cb_entries);
  void setCodebook(const cv::Mat &_cb_centers, const std::vector<std::vector<std::pair<int, int>>> &_cb_entries);
  void queryViewRank(const cv::Mat &descriptors, std::vector<std::pair<int, int>> &view_rank);
  void queryMatches(const cv::Mat &descriptors, std::vector<std::vector<cv::DMatch>> &matches,
                    bool sort_view_rank = true);

  inline const std::vector<std::vector<std::pair<int, int>>> &getEntries() const {
    return cb_entries;
  }
  inline const cv::Mat &getDescriptors() const {
    return cb_centers;
  }
  inline const std::vector<std::pair<int, int>> &getViewRank() const {
    return view_rank;
  }

  typedef SmartPtr<::v4r::CodebookMatcher> Ptr;
  typedef SmartPtr<::v4r::CodebookMatcher const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

}  //--END--

#endif

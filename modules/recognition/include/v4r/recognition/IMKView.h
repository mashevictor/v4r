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
 * @file RansacSolvePnPdepth.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KP_IMK_VIEW_HH
#define KP_IMK_VIEW_HH

#include <stdio.h>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <v4r/common/impl/DataMatrix2D.hpp>

namespace v4r {

class V4R_EXPORTS IMKView {
 public:
  unsigned object_id;
  std::vector<Eigen::Vector3f> points;
  std::vector<cv::KeyPoint> keys;

  DataMatrix2D<Eigen::Vector3f> cloud;
  cv::Mat_<float> weight_mask;
  std::vector<float> conf_desc;

  cv::Mat_<unsigned char> im_gray;
  IMKView() {}
  IMKView(const unsigned &_object_id) : object_id(_object_id) {}
};

}  //--END--

#endif

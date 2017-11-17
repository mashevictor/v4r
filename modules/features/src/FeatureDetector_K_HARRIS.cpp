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

#include "v4r/features/FeatureDetector_K_HARRIS.h"

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_K_HARRIS::FeatureDetector_K_HARRIS(const Parameter &_p) : FeatureDetector(K_HARRIS), param(_p) {
  imGOri.reset(new ComputeImGDescOrientations(param.goParam));
}

FeatureDetector_K_HARRIS::~FeatureDetector_K_HARRIS() {}

/***************************************************************************************/

/**
 * detect
 */
void FeatureDetector_K_HARRIS::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, CV_RGB2GRAY);
  else
    im_gray = image;

  cv::goodFeaturesToTrack(im_gray, pts, param.maxCorners, param.qualityLevel, param.minDistance, cv::Mat(), 3, true,
                          0.01);

  keys.resize(pts.size());
  for (unsigned i = 0; i < keys.size(); i++) {
    keys[i] = cv::KeyPoint(pts[i], param.winSize - 2);
  }

  imGOri->compute(im_gray, keys);
}
}

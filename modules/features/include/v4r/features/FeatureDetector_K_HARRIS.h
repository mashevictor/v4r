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

#ifndef KP_FEATURE_DETECTOR_HARRIS_HH
#define KP_FEATURE_DETECTOR_HARRIS_HH

#include <opencv2/features2d/features2d.hpp>
#include "ComputeImGDescOrientations.h"
#include "FeatureDetector.h"

namespace v4r {

class V4R_EXPORTS FeatureDetector_K_HARRIS : public FeatureDetector {
 public:
  class Parameter {
   public:
    int winSize;  // e.g. 32*32 window + 2px border = 34
    int maxCorners;
    double qualityLevel;  // 0.0001
    double minDistance;
    bool computeDesc;
    bool uprightDesc;
    ComputeImGDescOrientations::Parameter goParam;

    Parameter(int _winSize = 34, int _maxCorners = 5000, const double &_qualityLevel = 0.0002,
              const double &_minDistance = 1., bool _computeDesc = true, bool _uprightDesc = false,
              const ComputeImGDescOrientations::Parameter &_goParam = ComputeImGDescOrientations::Parameter())
    : winSize(_winSize), maxCorners(_maxCorners), qualityLevel(_qualityLevel), minDistance(_minDistance),
      computeDesc(_computeDesc), uprightDesc(_uprightDesc), goParam(_goParam) {}
  };

 private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray;

  std::vector<cv::Point2f> pts;

  ComputeImGDescOrientations::Ptr imGOri;

 public:
  FeatureDetector_K_HARRIS(const Parameter &_p = Parameter());
  ~FeatureDetector_K_HARRIS();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys);

  typedef SmartPtr<::v4r::FeatureDetector_K_HARRIS> Ptr;
  typedef SmartPtr<::v4r::FeatureDetector_K_HARRIS const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

}  //--END--

#endif

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

#include <v4r/features/FeatureDetector_KD_ORB.h>

#if CV_MAJOR_VERSION < 3
#define HAVE_OCV_2
#endif

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_KD_ORB::FeatureDetector_KD_ORB(const Parameter &_p) : FeatureDetector(KD_ORB), param(_p) {
// orb = new cv::ORB(10000, 1.2, 6, 13, 0, 2, cv::ORB::HARRIS_SCORE, 13); //31
// orb = new cv::ORB(1000, 1.44, 2, 17, 0, 2, cv::ORB::HARRIS_SCORE, 17);

#ifdef HAVE_OCV_2
  orb = new cv::ORB(param.nfeatures, param.scaleFactor, param.nlevels, param.patchSize, 0, 2, cv::ORB::HARRIS_SCORE,
                    param.patchSize);
#else
  orb = cv::ORB::create(param.nfeatures, param.scaleFactor, param.nlevels, 31, 0, 2, cv::ORB::HARRIS_SCORE,
                        param.patchSize);
#endif
}

FeatureDetector_KD_ORB::~FeatureDetector_KD_ORB() {}

/***************************************************************************************/

/**
 * detect
 * descriptors is a cv::Mat_<unsigned char>
 */
void FeatureDetector_KD_ORB::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, CV_RGB2GRAY);
  else
    im_gray = image;

#ifdef HAVE_OCV_2
  (*orb)(im_gray, cv::Mat(), keys, descriptors);
#else
  orb->detectAndCompute(im_gray, cv::Mat(), keys, descriptors);
#endif
}

/**
 * detect
 */
void FeatureDetector_KD_ORB::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, CV_RGB2GRAY);
  else
    im_gray = image;

  orb->detect(im_gray, keys);
}

/**
 * detect
 */
void FeatureDetector_KD_ORB::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
  if (image.type() != CV_8U)
    cv::cvtColor(image, im_gray, CV_RGB2GRAY);
  else
    im_gray = image;

#ifdef HAVE_OCV_2
  (*orb)(im_gray, cv::Mat(), keys, descriptors, true);
#else
  orb->detectAndCompute(im_gray, cv::Mat(), keys, descriptors, true);
#endif
}
}

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

#ifndef KP_FEATURE_DETECTOR_HH
#define KP_FEATURE_DETECTOR_HH

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <v4r/core/macros.h>
#include <v4r/common/impl/SmartPtr.hpp>


namespace v4r 
{

class V4R_EXPORTS FeatureDetector
{
public:
  enum Type
  {
    K_MSER,
    K_HARRIS,
    KD_CVSURF,
    KD_CVSIFT,
    KD_SIFTGPU,
    D_FREAK,
    KD_ORB,
    KD_FAST_IMGD,
    KD_PSURF,
    KD_MSER_IMGD,
    KD_HARRIS_IMGD,
    KD_PSURF_FREAK,
    KD_PSURF_IMGD,
    KD_FAST_PSURF,
    KD_FAST_SIFTGPU,
    KD_CVSURF_FREAK,
    KD_CVSURF_IMGD,
    KD_FAST_CVSURF,
    KD_SIFTGPU_IMGD,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

private:
  Type type;

public:
  FeatureDetector(Type _type=UNDEF) : type(_type) {}
  virtual ~FeatureDetector() {}

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) { 
      (void)image;
      (void)keys;
      (void)descriptors;
      std::cout<<"[FeatureDetector::detect] Not implemented!]"<<std::endl; };

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys) {
      (void)image;
      (void)keys;
    std::cout<<"[FeatureDetector::detect] Not implemented!]"<<std::endl; }

   virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
      (void)image;
      (void)keys;
      (void)descriptors;
    std::cout<<"[FeatureDetector::extract] Not implemented!]"<<std::endl; }
 

  Type getType() {return type;}

  typedef SmartPtr< ::v4r::FeatureDetector> Ptr;
  typedef SmartPtr< ::v4r::FeatureDetector const> ConstPtr;
};

}

#endif


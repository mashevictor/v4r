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

#ifndef V4R_COMPUTE_GRADIENT_DESCRIPTORS_HH
#define V4R_COMPUTE_GRADIENT_DESCRIPTORS_HH

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ImGradientDescriptor.h"

namespace v4r 
{

class V4R_EXPORTS ComputeImGradientDescriptors
{
public:
  class Parameter
  {
  public:
    int win_size;
    ImGradientDescriptor::Parameter ghParam;
    Parameter(int _win_size=34, 
      const ImGradientDescriptor::Parameter &_ghParam=ImGradientDescriptor::Parameter())
    : win_size(_win_size), ghParam(_ghParam) {}
  };

private:
  Parameter param;

  int h_win;

public:
 

  ComputeImGradientDescriptors(const Parameter &p=Parameter());
  ~ComputeImGradientDescriptors();

  void compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::Point2f> &pts, 
        cv::Mat &descriptors);
  void compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::KeyPoint> &keys, 
        cv::Mat &descriptors);
  //void compute(const cv::Mat_<unsigned char> &image, const std::vector<AffKeypoint> &keys, 
  //      cv::Mat &descriptors);




  typedef SmartPtr< ::v4r::ComputeImGradientDescriptors> Ptr;
  typedef SmartPtr< ::v4r::ComputeImGradientDescriptors const> ConstPtr;

};


/*************************** INLINE METHODES **************************/


} //--END--

#endif


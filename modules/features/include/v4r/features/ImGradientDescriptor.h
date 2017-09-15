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

#ifndef V4R_IM_GRADIENT_DESCRIPTOR_HH
#define V4R_IM_GRADIENT_DESCRIPTOR_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <stdexcept>
#include <v4r/common/impl/SmartPtr.hpp>

namespace v4r 
{

class V4R_EXPORTS ImGradientDescriptor
{
public:
  class Parameter
  {
  public:
    bool smooth;
    float sigma; //1.6
    float thrCutDesc;
    bool gauss_lin;
    bool computeRootGD;  // L1 norm and square root => euc dist = hellinger dist
    bool normalize;
    Parameter(bool _smooth=true, float _sigma=1.6, float _thrCutDesc=.2, bool _gauss_lin=false,
      bool _computeRootGD=true, bool _normalize=true)
    : smooth(_smooth), sigma(_sigma), thrCutDesc(_thrCutDesc), gauss_lin(_gauss_lin), 
      computeRootGD(_computeRootGD), normalize(_normalize) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_smooth;
  cv::Mat_<short> im_dx, im_dy;
  cv::Mat_<float> lt_gauss;

  void ComputeGradients(const cv::Mat_<unsigned char> &im);
  void ComputeDescriptor(std::vector<float> &desc, const cv::Mat_<float> &weight);
  void ComputeDescriptorInterpolate(std::vector<float> &desc, const cv::Mat_<float> &weight);
  void ComputeLTGauss(const cv::Mat_<unsigned char> &im);
  void ComputeLTGaussCirc(const cv::Mat_<unsigned char> &im);
  void ComputeLTGaussLin(const cv::Mat_<unsigned char> &im);
  void Normalize(std::vector<float> &desc);
  void Cut(std::vector<float> &desc);

  inline int sign(const float &v);


public:

  ImGradientDescriptor(const Parameter &p=Parameter());
  ~ImGradientDescriptor();

  void compute(const cv::Mat_<unsigned char> &im, std::vector<float> &desc);
  void compute(const cv::Mat_<unsigned char> &im,const cv::Mat_<float> &weight, std::vector<float> &desc);

  typedef SmartPtr< ::v4r::ImGradientDescriptor> Ptr;
  typedef SmartPtr< ::v4r::ImGradientDescriptor const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline int ImGradientDescriptor::sign(const float &v)
{
  if (v<0.) return -1;
  return +1;
}


} //--END--

#endif


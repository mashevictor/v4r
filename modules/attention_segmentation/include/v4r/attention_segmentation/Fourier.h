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
 * @file Fourier.h
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use fourier filter to compare surface texture.
 */

#ifndef SURFACE_FOURIER_H
#define SURFACE_FOURIER_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include "v4r/attention_segmentation//SurfaceModel.h"

namespace v4r
{

class V4R_EXPORTS Fourier
{
public:
  
protected:

private:
  
  bool computed;
  bool have_image;
  cv::Mat image;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  int width, height;
  
  int N;            // Number of neighbors
  int kmax;         // maximum number of discrete fourier transformation coefficient
  int nbins;        // number of histogram bins
  int binWidth;     // Width of one bin (32 width => 8x32 = 256)
  int binStretch;   // Stretch factor for bins of higher order (k=1,...)
  
public:
  
  typedef boost::shared_ptr<Fourier> Ptr;
  
  Fourier();
  ~Fourier();
  
  /** Set input point cloud **/
  // sets input cloud
  //
  void setInputImage(cv::Mat &_image);
  // sets indices
  void setIndices(pcl::PointIndices::Ptr _indices);
  void setIndices(std::vector<int> &_indices);
  void setIndices(cv::Rect rect);

  bool getComputed() {return computed;};
  
  /** Compute the texture **/
  void compute();
  
  /** Compare surface texture **/
  V4R_EXPORTS double compare(Fourier::Ptr f);

  /** Check the results visually on images for each dft-component **/
//   void check();

  double *bins;
  uchar *dft;         // dft results for kmax = 5 coefficients
  bool *used;
  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif


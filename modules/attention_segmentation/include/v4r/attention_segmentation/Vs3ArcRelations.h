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
 * @file Vs3ArcRelations.h
 * @author Richtsfeld
 * @date March 2013
 * @version 0.1
 * @brief Relations based on arc groupings from canny edges.
 */

#ifndef SURFACE_VS3_ARC_RELATION_H
#define SURFACE_VS3_ARC_RELATION_H

// #include <omp.h>
// #include <vector>
// #include <cstdio>
// 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.h>
#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.h"
// #include "v4r/SurfaceBoundary/VisionCore.h"
// 
// #include "ColorHistogram3D.h"
// #include "Texture.h"
// #include "Fourier.h"
// #include "Gabor.h"
// #include "BoundaryRelations.h"
// #include "ContourNormalsDistance.h"


namespace surface
{
  
class Vs3ArcRelations
{
public:
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  
  bool have_input_image;
  bool have_view;
  bool preprocessed;
  
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;             ///< Input cloud
  cv::Mat edges_image;                                                ///< Canny edge image
  v4r::View *view;                                          ///< Surface models

  cv::Mat_<cv::Vec3b> matImage;                                 ///< Image as Mat
  IplImage *iplImage;                                           ///< Image as IplImage

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);
  
public:
  Vs3ArcRelations();
  ~Vs3ArcRelations();

  /** Set input point cloud **/
  void setInputImage(cv::Mat &_matImage);
  
  /** Set input surface patches **/
  void setView(v4r::View *_view);

  /** Preprocess the vs3 primitives **/
  void preprocess();

  /** Compute relations for the segmenter **/
//   void computeRelations();

};

/*************************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int Vs3ArcRelations::GetIdx(short x, short y)
{
  return y*view->width + x;
}

/** Return x coordinate for index **/
inline short Vs3ArcRelations::X(int idx)
{
  return idx%view->width;
}

/** Return y coordinate for index **/
inline short Vs3ArcRelations::Y(int idx)
{
  return idx/view->width;
}

} //--END--

#endif


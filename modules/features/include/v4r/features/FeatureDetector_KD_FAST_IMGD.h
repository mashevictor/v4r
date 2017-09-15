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

#ifndef KP_KEYPOINT_DETECTOR_ORB_IMGDESC_HH
#define KP_KEYPOINT_DETECTOR_ORB_IMGDESC_HH

#include <opencv2/features2d/features2d.hpp>
#include <v4r/features/FeatureDetector.h>
#include <v4r/features/ComputeImGradientDescriptors.h>
#include <v4r/features/FeatureSelection.h>



namespace v4r 
{

class V4R_EXPORTS FeatureDetector_KD_FAST_IMGD : public FeatureDetector
{
public:
  class V4R_EXPORTS Parameter
  {
  public:
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int patchSize;
    int tiles;
    ComputeImGradientDescriptors::Parameter gdParam;
    bool do_feature_selection;

    Parameter(int _nfeatures=1000, float _scaleFactor=1.44, 
      int _nlevels=2, int _patchSize=17, int _tiles=1,
      const ComputeImGradientDescriptors::Parameter &_gdParam=ComputeImGradientDescriptors::Parameter(),
      bool _do_feature_selection=false)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), 
      nlevels(_nlevels), patchSize(_patchSize), tiles(_tiles),
      gdParam(_gdParam),
      do_feature_selection(_do_feature_selection) {}
  };

private:
  Parameter param;

  const static int PATCH_SIZE = 15;

  int tile_size_w, tile_size_h;

  cv::Mat_<unsigned char> im_gray;  

  std::vector<cv::Point2f> pts;

  cv::Ptr<cv::ORB> orb;
  ComputeImGradientDescriptors::Ptr imGDesc;

  FeatureSelection::Ptr fs;

  inline void getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect);

public:
  FeatureDetector_KD_FAST_IMGD(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_FAST_IMGD();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 


  typedef SmartPtr< ::v4r::FeatureDetector_KD_FAST_IMGD> Ptr;
  typedef SmartPtr< ::v4r::FeatureDetector_KD_FAST_IMGD const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

/**
 * getExpandedRect
 */
inline void FeatureDetector_KD_FAST_IMGD::getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect)
{
  int border = PATCH_SIZE;

  int x1 = u*tile_size_w;
  int y1 = v*tile_size_h;
  int x2 = x1 + tile_size_w;
  int y2 = y1 + tile_size_h;

  x1 -= border;
  y1 -= border;
  x2 += border;
  y2 += border;

  if (x1<0) x1 = 0;
  if (y1<0) y1 = 0;
  if (x2>=cols) x2 = cols-1;
  if (y2>=rows) y2 = rows-1;

  rect = cv::Rect(x1, y1, x2-x1, y2-y1);
}


} //--END--

#endif


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

#ifndef FRINTROP_SALIENCYMAP_HPP
#define FRINTROP_SALIENCYMAP_HPP

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/BaseMap.h"
#include "v4r/attention_segmentation/ColorMap.h"

namespace v4r {

class V4R_EXPORTS FrintropSaliencyMap : public BaseMap {
 public:
  FrintropSaliencyMap();   //
  ~FrintropSaliencyMap();  //

  void setNumberOfOrientations(int numberOfOrientations_);  //
  int getNumberOfOrientations();                            //

  virtual int calculate();
  virtual void reset();  //
  virtual void print();  //

 private:
  cv::Mat R, G, B, Y, I;
  int numberOfOrientations;

  void initializePyramid(FrintropPyramid::Ptr pyramid, cv::Mat &IM, bool onSwitch_);  //
  void initializePyramid(SimplePyramid::Ptr pyramid, cv::Mat &IM);                    //
  void createColorChannels();                                                         //
  int createFeatureMapsI(FrintropPyramid::Ptr pyramid);                               //
  int createFeatureMapsO(SimplePyramid::Ptr pyramid, float angle);                    //

 protected:
  virtual int checkParameters();  //
};

}  // v4r

#endif  // FRINTROP_SALIENCYMAP_HPP

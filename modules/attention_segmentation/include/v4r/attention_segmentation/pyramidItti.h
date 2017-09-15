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



#ifndef PYRAMID_ITTI_HPP
#define PYRAMID_ITTI_HPP

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/pyramidBase.h"

namespace v4r
{

class V4R_EXPORTS IttiPyramid: public BasePyramid
{
public:
  IttiPyramid();
  virtual ~IttiPyramid();
  typedef boost::shared_ptr<IttiPyramid> Ptr;

  void setLowestC(int lowest_c_);
  int getLowestC();

  void setHighestC(int highest_c_);
  int getHighestC();

  void setSmallestCS(int smallest_cs_);
  int getSmallestCS();

  void setLargestCS(int largest_cs_);
  int getLargestCS();

  void setNumberOfFeatures(int number_of_features_);
  int getNumberOfFeatures();

  void setChangeSign(bool changeSign_);
  bool getChangeSign();

  virtual void reset();
  virtual void print();
  virtual void combinePyramid(bool standard = false);
  
private:
  int                  lowest_c;
  int                  highest_c;
  int                  smallest_cs;
  int                  largest_cs;
  int                  number_of_features;
  bool                 changeSign;
  
  std::vector<cv::Mat> pyramidConspicuities;

  virtual void checkLevels();
};

}
#endif //PYRAMID_ITTI_HPP

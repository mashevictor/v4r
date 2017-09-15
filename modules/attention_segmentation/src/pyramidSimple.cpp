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



#include "v4r/attention_segmentation/pyramidSimple.h"

namespace v4r
{

SimplePyramid::SimplePyramid():
BasePyramid()
{
  reset();
}

SimplePyramid::~SimplePyramid()
{
}

void SimplePyramid::reset()
{
  BasePyramid::reset();

  pyramidName = "SimplePyramid";
}

void SimplePyramid::combinePyramid(bool standard)
{
  int sm_width = pyramidImages.at(sm_level).cols;
  int sm_height = pyramidImages.at(sm_level).rows;
  if(combination_type == AM_COMB_MUL)
  {
    map = cv::Mat_<float>::ones(sm_height,sm_width);
  }
  else
  {
    map = cv::Mat_<float>::zeros(sm_height,sm_width);
  }

  for(int i = start_level; i <= max_level; ++i)
  {
    cv::Mat temp;
    
//     cv::imshow("pyramidFeatures.at(i)",pyramidFeatures.at(i));
//     cv::waitKey(-1);
    
    if(standard)
    {
      cv::resize(pyramidFeatures.at(i),temp,map.size());
    }
    else
    {
      v4r::scaleImage(pyramidFeatures,pyramidFeatures.at(i),temp,i,sm_level);
    }
    
//     cv::imshow("temp",temp);
//     cv::waitKey(-1);
    
    v4r::normalize(temp,normalization_type);
//     cv::imshow("temp",temp);
//     cv::waitKey(-1);
    combineConspicuityMaps(map,temp);
    
//     cv::imshow("map",map);
//     cv::waitKey(-1);
  }

  double maxValue, minValue;
  cv::minMaxLoc(map,&minValue,&maxValue);
  max_map_value = maxValue;
  v4r::normalize(map,normalization_type);
  
  calculated = true;
}

}

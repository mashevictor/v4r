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



#include "v4r/attention_segmentation/MapsCombination.h"

namespace v4r
{

int CombineMaps(std::vector<cv::Mat> &maps, cv::Mat &combinedMap, int combination_type, int normalization_type)
{
  if(!maps.size())
  {
    printf("[ERROR] CombineMaps: there should be at least one map!\n");
    return(AM_ZEROSIZE);
  }

  switch(combination_type)
  {
    case AM_SUM:
      combinedMap = cv::Mat_<float>::zeros(maps.at(0).rows,maps.at(0).cols);
      for(unsigned int i = 0; i < maps.size(); ++i)
      {
        cv::add(maps.at(i),combinedMap,combinedMap);
      }
      combinedMap = combinedMap / maps.size();
      v4r::normalize(combinedMap,normalization_type);
      //v4r::normalize(combinedMap,v4r::NT_NONE);
      return(AM_OK);
    case AM_MUL:
      combinedMap = cv::Mat_<float>::ones(maps.at(0).rows,maps.at(0).cols);
      for(unsigned int i = 0; i < maps.size(); ++i)
      {
        cv::multiply(maps.at(i),combinedMap,combinedMap);
      }
      v4r::normalize(combinedMap,normalization_type);
      //v4r::normalize(combinedMap,v4r::NT_NONE);
      return(AM_OK);
    case AM_MIN:
      combinedMap = cv::Mat_<float>::ones(maps.at(0).rows,maps.at(0).cols);
      for(unsigned int i = 0; i < maps.size(); ++i)
      {
        combinedMap = cv::min(maps.at(i),combinedMap);
      }
      v4r::normalize(combinedMap,normalization_type);
      //v4r::normalize(combinedMap,v4r::NT_NONE);
      return(AM_OK);
    case AM_MAX:
      combinedMap = cv::Mat_<float>::zeros(maps.at(0).rows,maps.at(0).cols);
      for(unsigned int i = 0; i < maps.size(); ++i)
      {
        combinedMap = cv::max(maps.at(i),combinedMap);
      }
      v4r::normalize(combinedMap,normalization_type);
      //v4r::normalize(combinedMap,v4r::NT_NONE);
      return(AM_OK);
    default:
      combinedMap = cv::Mat_<float>::zeros(maps.at(0).rows,maps.at(0).cols);
      return(AM_PARAMETERS);
  }
}

}

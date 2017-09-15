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



#include "v4r/attention_segmentation/normalization.h"

namespace v4r
{

void computeLocalMax(cv::Mat &image, int &numLocalMax, float &averageLocalMax, float threshold)
{
  averageLocalMax = 0;
  numLocalMax = 0;

  for (int r = 0; r < image.rows; ++r)
  {
    for (int c = 0; c < image.rows; ++c)
    {
      if (c-1 >= 0)
      {
        if (image.at<float>(r,c-1) >= image.at<float>(r,c))
        {
          continue;
        }
        if (r-1 >=0)
        {
          if (image.at<float>(r-1,c-1) >= image.at<float>(r,c))
          {
            continue;
          }
        }
        if (r+1 < image.rows)
        {
          if (image.at<float>(r+1,c-1) >= image.at<float>(r,c))
          {
            continue;
          }
        }
      }
      if (c+1 < image.cols)
      {
        if (image.at<float>(r,c+1) >= image.at<float>(r,c))
        {
          continue;
        }
        if (r-1 >= 0)
        {
          if (image.at<float>(r-1,c+1) >= image.at<float>(r,c))
          {
            continue;
          }
        }
        if (r+1 < image.rows)
        {
          if (image.at<float>(r+1,c+1) >= image.at<float>(r,c))
          {
            continue;
          }
        }
      }
      if (r-1 >= 0)
      {
        if (image.at<float>(r-1,c) >= image.at<float>(r,c))
        {
          continue;
        }
      }
      if (r+1 < image.rows)
      {
        if (image.at<float>(r+1,c) >= image.at<float>(r,c))
        {
          continue;
        }
      }

      if(image.at<float>(r,c) > threshold)
      {
        averageLocalMax += image.at<float>(r,c);
        numLocalMax +=1;
      }
    
    }
  }
  
  if (numLocalMax > 0)
  {
    averageLocalMax /= numLocalMax;
  }
  
}

void normalizeNonMax(cv::Mat &map)
{
  cv::normalize(map,map,0,1,cv::NORM_MINMAX);
  //normalizeMin2Zero(map);
  int numLocalMax;
  float averageLocalMax;
  computeLocalMax(map,numLocalMax,averageLocalMax);
  float multiplier = (1-averageLocalMax)*(1-averageLocalMax);
  map = multiplier * map;
  //normalizeMax2One(map);
}

void normalizeFrintrop(cv::Mat &map)
{
  double maxValue, minValue;
  cv::minMaxLoc(map,&minValue,&maxValue);
  int numLocalMax;
  float averageLocalMax;
  computeLocalMax(map,numLocalMax,averageLocalMax,0.5*maxValue);
  if(numLocalMax > 0)
    map = map / sqrt((float)numLocalMax);
}

void normalizeMin2Zero(cv::Mat &map)
{
  if((map.rows <= 0) || (map.cols <= 0))
    return;
  
  float min = map.at<float>(0,0);
  float max = min;
  
  for(int r = 0; r < map.rows; ++r)
  {
    for(int c = 0; c < map.cols; ++c)
    {
      float value = map.at<float>(r,c);
      if(value > max)
      {
	max = value;
      }
      if((value > 0) && (value < min))
      {
	min = value;
      }
    }
  }

  for(int r = 0; r < map.rows; ++r)
  {
    for(int c = 0; c < map.cols; ++c)
    {
      float value = map.at<float>(r,c);
      if(value > 0)
      {
        map.at<float>(r,c) = (value - min)/(max-min);
      }
    }
  }

  return;
}

void normalizeMax2One(cv::Mat &map)
{
  double minVal, maxVal;
  cv::minMaxLoc(map,&minVal,&maxVal);
  if(maxVal > 0)
  {
    map = map / maxVal;
  }
}

void normalize(cv::Mat &map, int normalization_type, float newMaxValue, float newMinValue)
{
  switch(normalization_type)
  {
    case NT_NONE:
      cv::normalize(map,map,newMinValue,newMaxValue,cv::NORM_MINMAX);
      return;
    case NT_NONMAX:
      normalizeNonMax(map);
      return;
    case NT_FRINTROP_NORM:
      normalizeFrintrop(map);
      return;
    case NT_EMPTY:
      return;
    case NT_NONE_REAL:
      normalizeMin2Zero(map);
      return;
    case NT_MAX_DIVIDE:
      normalizeMax2One(map);
      return;
    default:
      cv::normalize(map,map,0,1,cv::NORM_MINMAX);
      return;
  }
}

} // namespace v4r 

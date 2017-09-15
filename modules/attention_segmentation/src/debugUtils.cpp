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



#include "v4r/attention_segmentation/debugUtils.h"
#include "v4r/attention_segmentation/convertions.h"

namespace v4r
{

void showImageForDebug(std::string name, const cv::Mat &image, int waitTime)
{
  cv::Mat showImage;
  image.copyTo(showImage);
  
  if(showImage.type() == CV_32FC1)
  {
    cv::normalize(showImage,showImage,0,1,cv::NORM_MINMAX);
    FloatMap2UcharMap(showImage,showImage);
  }
  
  cv::imshow(name,showImage);
  cv::waitKey(waitTime);
}

void printMat(std::string name, cv::Mat &mat)
{
  std::cerr << name << ":" << std::endl;
  for(int r = 0; r < mat.rows; ++r)
  {
    for(int c = 0; c < mat.cols; ++c)
    {
      std::cerr << mat.at<float>(r,c) << " ";
    }
    std::cerr << std::endl;
  }
}

void printParameter(const int type, std::string name, float value)
{
  //debug.print(type,"[variable] %s = %4.3f\n",name.c_str(),value);
  //std::cerr << name << " = " << value << std::endl;
}

//DebugPrints::Ptr debug = DebugPrints::Ptr(new DebugPrints());

DebugPrints::DebugPrints()
{
  DebugType = LIGHT_TIME_PRINT;
  pFile = 0;
  fileOpened = false;
}

DebugPrints::~DebugPrints()
{
  if(fileOpened)
    fclose(pFile);
}

void DebugPrints::setDebugType(int type)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  DebugType = type;
  debug_mutex.unlock();
}

void DebugPrints::openFile(const char *fileName)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  pFile = fopen (fileName , "a");
  debug_mutex.unlock();
}

void DebugPrints::closeFile()
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  fclose(pFile);
  pFile = 0;
  debug_mutex.unlock();
}

void DebugPrints::print(const int type, const char *fmt, ...)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  va_list ap1;
  if (DebugType >= type)
  {
    va_start(ap1, fmt);
    if(pFile)
      vfprintf(pFile,fmt, ap1);
    else
      vprintf(fmt, ap1);
    va_end(ap1);
  }
  debug_mutex.unlock();
}

//DebugPrints::Ptr DebugPrints::getDebug()
//{
  //if(debug == NULL)
  //{
    //DebugPrints::Ptr debug = DebugPrints::Ptr(new DebugPrints());
  //}
  //return debug;
//}

} //namespace v4r

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



#ifndef EPDEBUG_H
#define EPDEBUG_H

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/eputils_headers.h"

namespace v4r
{

/**
 * show image for debug
 * */
V4R_EXPORTS void showImageForDebug(std::string name, const cv::Mat &image, int waitTime = 5);
/**
 * print mat
 * */
V4R_EXPORTS void printMat(std::string name, cv::Mat &mat);
/**
 * print value
 * */
V4R_EXPORTS void printParameter(std::string name, float value);

// class to print
class DebugPrints
{
public:

  typedef boost::shared_ptr<DebugPrints> Ptr;
  typedef boost::shared_ptr<const DebugPrints> ConstPtr;
  
  typedef enum
  {
    NO_DEBUG_PRINT = 0,
    CRITICAL_PRINT = 1,
    LIGHT_TIME_PRINT = 2,
    FULL_TIME_PRINT = 3,
    FULL_DEBUG_PRINT = 4,
    RINGBUFFY_DEBUG_PRINT = 5
  } DEBUG_TYPE;

  ~DebugPrints();
  
  void setDebugType(int type);
  void print(const int type, const char *fmt, ...);
  void openFile(const char *fileName);
  void closeFile();

  //static DebugPrints::Ptr getDebug();

public:
  DebugPrints();
  
  int DebugType;
  FILE *pFile;
  bool fileOpened;

  boost::mutex debug_mutex;

  //static DebugPrints::Ptr debug;// = DebugPrints::Ptr(new DebugPrints());
};

//DebugPrints debug;

} //namespace v4r

#endif //EPDEBUG_H

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


#ifndef EPUTILS_MODULE_HEADERS_HPP
#define EPUTILS_MODULE_HEADERS_HPP

#include <string>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#define BOOST_FILESYSTEM_DEPRECATED

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Eigen>

#include <v4r/core/macros.h>

// #ifndef NOT_USE_PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "v4r/attention_segmentation/PCLPreprocessingXYZRC.h"
// #endif


namespace v4r
{
  
static const int dy8[8] = {-1,-1,-1,0,1,1,1,0};
static const int dx8[8] = {-1,0,1,1,1,0,-1,-1};

static const int dx4[4] = {-1,1,0,0};
static const int dy4[4] = {0,0,-1,1};

} //namespace v4r

#endif //EPUTILS_MODULE_HEADERS_HPP

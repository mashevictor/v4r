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



#ifndef EPCONNECTEDCOMPONENTS_H
#define EPCONNECTEDCOMPONENTS_H

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/eputils_headers.h"

namespace v4r 
{
  
struct ConnectedComponent {
  std::vector<cv::Point> points;
  std::vector<float> saliency_values;
  float average_saliency;
  ConnectedComponent();
};
/**
 * extracts connected components from the map using given threshold
 * map is considered to be in the range 0..1 with type CV_32F
 * */
V4R_EXPORTS void extractConnectedComponents(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th = 0.1);
V4R_EXPORTS void extractConnectedComponents(cv::Mat map, std::vector<ConnectedComponent> &connected_components, cv::Point attention_point, float th = 0.1);

/**
 * extracts connected components from the map using given threshold
 * map is considered to be in the range 0..1 with type CV_32F
 * */
//void extractConnectedComponents2(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th = 0.1);

/**
 * draws single connected component over the image
 */
V4R_EXPORTS void drawConnectedComponent(ConnectedComponent component, cv::Mat &image, cv::Scalar color);

/**
 * draws connected components over the image
 */
V4R_EXPORTS void drawConnectedComponents(std::vector<ConnectedComponent> components, cv::Mat &image, cv::Scalar color);

} //namespace v4r 

#endif // EPCONNECTEDCOMPONENTS_H

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



#ifndef EPDRAWUTILS_H
#define EPDRAWUTILS_H

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/eputils_headers.h"
#include "v4r/attention_segmentation/connectedComponents.h"

namespace v4r
{

/**
 * draw one segmentation masks
 * */
V4R_EXPORTS void drawSegmentationMask(cv::Mat &image, cv::Mat mask, cv::Scalar color, int line_width = 2);
  
/**
 * draw a banch of segmentation masks
 * */
V4R_EXPORTS void drawSegmentationMasks(cv::Mat &image, std::vector<cv::Mat> &masks, int line_width = 2);

/**
 * draw a segmentation masks and attetnion points
 * */
V4R_EXPORTS void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<std::vector<cv::Point> > &contours, bool drawAttentionPoints = true,
                             bool drawSegmentationResults = true, bool drawLines = false, unsigned int num = -1);

/**
 * draws segmentation and attention results
 * */
V4R_EXPORTS void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<cv::Mat> &binMasks, std::vector<std::vector<cv::Point> > &contours,
                             bool drawAttentionPoints, bool drawSegmentationResults);
V4R_EXPORTS void drawSegmentationResults(cv::Mat &image, cv::Point p1, cv::Mat &masks, bool drawAttentionPoints, bool drawSegmentationResults, int num);

//revision
/**
 * draws attention points
 * */
V4R_EXPORTS void drawAttentionPoints(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                         unsigned int maxNumber = 0, bool connect_points = false);
//end revision

/**
 * draws path through the map
 */
V4R_EXPORTS void drawPath(cv::Mat &image, std::vector<cv::Point> &path, cv::Mat &mapx, cv::Mat &mapy);

/**
 * draws line
 */
V4R_EXPORTS void drawLine(cv::Mat &image, std::vector<cv::Point> points, cv::Scalar color = cv::Scalar(0));

} //namespace v4r

#endif //EPDRAWUTILS_H

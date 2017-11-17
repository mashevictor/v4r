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

#include "v4r/attention_segmentation/MSR.h"

namespace v4r {

void winnerToImgCoords(cv::Point&, cv::Point&, int);
void defaultParamsMSR(MRSParams&);
void detectMSR(std::vector<cv::Point>&, cv::Mat, MRSParams);

void winnerToImgCoords(cv::Point& p_new, cv::Point& p, int mapLevel) {
  float x = p.x;
  float y = p.y;
  p_new.x = (int)(x * (pow(2, mapLevel - 1)));
  p_new.y = (int)(y * (pow(2, mapLevel - 1)));
}

void defaultParamsMSR(MRSParams& params) {
  params.th = 0.25;
  params.mapLevel = 5;
  params.useMorphologyOpenning = false;
}

void detectMSR(std::vector<cv::Point>& centers, cv::Mat map_, MRSParams params) {
  assert(map_.type() == CV_32FC1);

  cv::Mat map, temp;
  map_.copyTo(temp);

  // perform morphological operations if necessary
  if (params.useMorphologyOpenning) {
    // calculate the size of the kernel
    int kernel_size = pow(2, params.mapLevel - 1);
    cv::Mat element = cv::Mat_<uchar>::ones(kernel_size, kernel_size);
    cv::erode(map_, temp, element);
  }

  cv::resize(temp, map, cv::Size(map_.cols / (pow(2, params.mapLevel - 1)), map_.rows / (pow(2, params.mapLevel - 1))));

  double maxVal = 0;
  cv::Point maxLoc;
  cv::minMaxLoc(map, 0, &maxVal, 0, &maxLoc);

  while (maxVal > 0) {
    cv::Point maxLoc_new;
    winnerToImgCoords(maxLoc_new, maxLoc, params.mapLevel);
    centers.push_back(maxLoc_new);

    float maxValTh = (1 - params.th) * maxVal;

    std::list<cv::Point> points;
    points.push_back(maxLoc);
    cv::Mat used = cv::Mat_<uchar>::zeros(map.rows, map.cols);
    used.at<uchar>(maxLoc.y, maxLoc.x) = 1;
    map.at<float>(maxLoc.y, maxLoc.x) = 0;
    while (points.size()) {
      cv::Point p = points.front();
      points.pop_front();

      if (((p.x + 1) < map.cols) && (!used.at<uchar>(p.y, p.x + 1)) && (map.at<float>(p.y, p.x + 1) >= maxValTh)) {
        points.push_back(cv::Point(p.x + 1, p.y));
        used.at<uchar>(p.y, p.x + 1) = 1;
        map.at<float>(p.y, p.x + 1) = 0;
        // count++;
      }
      if (((p.x - 1) >= 0) && (!used.at<uchar>(p.y, p.x - 1)) && (map.at<float>(p.y, p.x - 1) >= maxValTh)) {
        points.push_back(cv::Point(p.x - 1, p.y));
        used.at<uchar>(p.y, p.x - 1) = 1;
        map.at<float>(p.y, p.x - 1) = 0;
        // count++;
      }
      if (((p.y + 1) < map.rows) && (!used.at<uchar>(p.y + 1, p.x)) && (map.at<float>(p.y + 1, p.x) >= maxValTh)) {
        points.push_back(cv::Point(p.x, p.y + 1));
        used.at<uchar>(p.y + 1, p.x) = 1;
        map.at<float>(p.y + 1, p.x) = 0;
        // count++;
      }
      if (((p.y - 1) >= 0) && (!used.at<uchar>(p.y - 1, p.x)) && (map.at<float>(p.y - 1, p.x) >= maxValTh)) {
        points.push_back(cv::Point(p.x, p.y - 1));
        used.at<uchar>(p.y - 1, p.x) = 1;
        map.at<float>(p.y - 1, p.x) = 0;
        // count++;
      }
    }
    cv::minMaxLoc(map, 0, &maxVal, 0, &maxLoc);

    //     cv::imshow("map",map);
    //     cv::waitKey();
  }
}

}  // namespace v4r

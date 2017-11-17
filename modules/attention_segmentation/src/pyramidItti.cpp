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

#include "v4r/attention_segmentation/pyramidItti.h"

namespace v4r {

IttiPyramid::IttiPyramid() : BasePyramid() {
  reset();
}

IttiPyramid::~IttiPyramid() {}

void IttiPyramid::reset() {
  BasePyramid::reset();

  lowest_c = 2;
  highest_c = 4;
  smallest_cs = 3;
  largest_cs = 4;
  number_of_features = 0;
  changeSign = false;

  pyramidName = "IttiPyramid";
}

void IttiPyramid::setLowestC(int lowest_c_) {
  lowest_c = lowest_c_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: lowest_c is set to: %d\n", pyramidName.c_str(), lowest_c);
}

int IttiPyramid::getLowestC() {
  return (lowest_c);
}

void IttiPyramid::setHighestC(int highest_c_) {
  highest_c = highest_c_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: highest_c is set to: %d\n", pyramidName.c_str(), highest_c);
}

int IttiPyramid::getHighestC() {
  return (highest_c);
}

void IttiPyramid::setSmallestCS(int smallest_cs_) {
  smallest_cs = smallest_cs_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: smallest_cs is set to: %d\n", pyramidName.c_str(), smallest_cs);
}

int IttiPyramid::getSmallestCS() {
  return (smallest_cs);
}

void IttiPyramid::setLargestCS(int largest_cs_) {
  largest_cs = largest_cs_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: largest_cs is set to: %d\n", pyramidName.c_str(), largest_cs);
}

int IttiPyramid::getLargestCS() {
  return (largest_cs);
}

void IttiPyramid::setNumberOfFeatures(int number_of_features_) {
  number_of_features = number_of_features_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: number_of_features is set to: %d\n", pyramidName.c_str(), number_of_features);
}

int IttiPyramid::getNumberOfFeatures() {
  return (number_of_features);
}

void IttiPyramid::setChangeSign(bool changeSign_) {
  changeSign = changeSign_;
  haveImage = false;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: changeSign is set to: %s\n", pyramidName.c_str(), changeSign ? "yes" : "no");
}

bool IttiPyramid::getChangeSign() {
  return (changeSign);
}

void IttiPyramid::print() {
  BasePyramid::print();

  printf("[PyramidParameters]: lowest_c               = %d\n", lowest_c);
  printf("[PyramidParameters]: highest_c              = %d\n", highest_c);
  printf("[PyramidParameters]: smallest_cs            = %d\n", smallest_cs);
  printf("[PyramidParameters]: largest_cs             = %d\n", largest_cs);
  printf("[PyramidParameters]: number_of_features     = %d\n", number_of_features);
  printf("[PyramidParameters]: changeSign             = %s\n", changeSign ? "yes" : "no");
}

void IttiPyramid::checkLevels() {
  start_level = 0;
  max_level = highest_c + largest_cs;

  int nw = width;
  int nh = height;
  int current_level = 0;
  while (((nw / 2) >= 1) && ((nh / 2) >= 1)) {
    ++current_level;
    nw = nw / 2;
    nh = nh / 2;
  }

  max_level = (current_level > max_level ? max_level : current_level);

  if (max_level < highest_c + largest_cs)
    largest_cs -= (highest_c + largest_cs - max_level);

  if (largest_cs < smallest_cs)
    largest_cs = smallest_cs = 0;

  if (sm_level > max_level) {
    sm_level = max_level;
  }

  number_of_features = (highest_c - lowest_c + 1) * (largest_cs - smallest_cs + 1);
}

void IttiPyramid::combinePyramid(bool standard) {
  calculated = false;

  pyramidConspicuities.clear();
  pyramidConspicuities.resize(number_of_features);

  std::vector<int> trueLevel;

  for (int i = lowest_c; i <= highest_c; ++i) {
    for (int j = smallest_cs; j <= largest_cs; ++j) {
      int current = (largest_cs - smallest_cs + 1) * (i - lowest_c) + (j - smallest_cs);
      if (current < number_of_features) {
        cv::Mat temp;
        if (standard) {
          resize(pyramidFeatures.at(i + j), temp, cv::Size(pyramidFeatures.at(i).cols, pyramidFeatures.at(i).rows));
        } else {
          v4r::scaleImage(pyramidFeatures, pyramidFeatures.at(i + j), temp, i + j, i);
        }

        //         cv::imshow("temp",temp);
        //         cv::waitKey(-1);

        // 	if(changeSign)
        // 	  temp = -1 * temp;

        cv::absdiff(pyramidFeatures.at(i), temp, pyramidConspicuities.at(current));
        v4r::normalize(pyramidConspicuities.at(current), normalization_type);
        trueLevel.push_back(i);
      }
    }
  }

  if (combination_type == AM_COMB_MUL) {
    map = cv::Mat_<float>::ones(pyramidImages.at(sm_level).rows, pyramidImages.at(sm_level).cols);
  } else {
    map = cv::Mat_<float>::zeros(pyramidImages.at(sm_level).rows, pyramidImages.at(sm_level).cols);
  }

  for (int i = 0; i < number_of_features; ++i) {
    cv::Mat temp;
    if (standard) {
      cv::resize(pyramidConspicuities.at(i), temp,
                 cv::Size(pyramidImages.at(sm_level).cols, pyramidImages.at(sm_level).rows));
    } else {
      v4r::scaleImage(pyramidImages, pyramidConspicuities.at(i), temp, trueLevel.at(i), sm_level);
    }
    //     cv::imshow("temp",temp);
    //     cv::waitKey(-1);
    combineConspicuityMaps(map, temp);
  }

  v4r::normalize(map, normalization_type);

  calculated = true;
}
}

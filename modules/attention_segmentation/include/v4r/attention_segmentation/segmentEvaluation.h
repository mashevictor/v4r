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

#ifndef EPEVALUATION_SEGMENT_HPP
#define EPEVALUATION_SEGMENT_HPP

#include <v4r/core/macros.h>
#include "v4r/attention_segmentation/headers.h"

namespace EPEvaluation {

void printSegmentationEvaluation(std::string output_filename, std::string base_name, std::vector<long int> &tp,
                                 std::vector<long int> &fp, std::vector<long int> &fn, std::vector<bool> &used,
                                 std::vector<int> &objNumber);
V4R_EXPORTS void evaluate(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l, cv::Mat &mask, std::string base_name,
                          std::string output_filename);
V4R_EXPORTS void evaluate(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l, cv::Mat &mask, cv::Point attention_point,
                          std::string base_name, std::string output_filename);
V4R_EXPORTS void evaluate(const cv::Mat &ground_truth_image, cv::Mat &mask, cv::Point attention_point,
                          std::string base_name, std::string output_filename);
void evaluateSegmentation(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l, cv::Mat &mask, std::vector<long int> &tp,
                          std::vector<long int> &fp, std::vector<long int> &fn, std::vector<bool> &used,
                          std::vector<int> &objNumber);
void evaluateSegmentation(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_cloud_l, cv::Mat &mask, cv::Point attention_point,
                          std::vector<long int> &tp, std::vector<long int> &fp, std::vector<long int> &fn,
                          std::vector<bool> &used, std::vector<int> &objNumber);
void evaluateSegmentation(const cv::Mat &ground_truth_image, cv::Mat &mask, cv::Point attention_point,
                          std::vector<long int> &tp, std::vector<long int> &fp, std::vector<long int> &fn,
                          std::vector<bool> &used, std::vector<int> &objNumber);

}  // namespace EPEvaluation

#endif  // EPEVALUATION_SEGMENT_HPP

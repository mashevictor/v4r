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

/**
 * @file all_headers.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */
#pragma once

#include <v4r/keypoints/types.h>

#include <v4r/keypoints/harris3d_keypoint_extractor.h>
#include <v4r/keypoints/iss_keypoint_extractor.h>
#include <v4r/keypoints/narf_keypoint_extractor.h>
#include <v4r/keypoints/uniform_sampling_extractor.h>

namespace v4r {

template <typename PointT>
std::vector<typename KeypointExtractor<PointT>::Ptr> initKeypointExtractors(int method,
                                                                            std::vector<std::string> &params) {
  std::vector<typename KeypointExtractor<PointT>::Ptr> keypoint_extractor;

  if (method & KeypointType::UniformSampling) {
    UniformSamplingExtractorParameter param;
    params = param.init(params);
    typename UniformSamplingExtractor<PointT>::Ptr ke(new UniformSamplingExtractor<PointT>(param));
    keypoint_extractor.push_back(boost::dynamic_pointer_cast<KeypointExtractor<PointT>>(ke));
  }
  if (method & KeypointType::ISS) {
    IssKeypointExtractorParameter param;
    params = param.init(params);
    typename IssKeypointExtractor<PointT>::Ptr ke(new IssKeypointExtractor<PointT>(param));
    keypoint_extractor.push_back(boost::dynamic_pointer_cast<KeypointExtractor<PointT>>(ke));
  }
  if (method & KeypointType::NARF) {
    NarfKeypointExtractorParameter param;
    params = param.init(params);
    typename NarfKeypointExtractor<PointT>::Ptr ke(new NarfKeypointExtractor<PointT>(param));
    keypoint_extractor.push_back(boost::dynamic_pointer_cast<KeypointExtractor<PointT>>(ke));
  }
  if (method & KeypointType::HARRIS3D) {
    Harris3DKeypointExtractorParameter param;
    params = param.init(params);
    typename Harris3DKeypointExtractor<PointT>::Ptr ke(new Harris3DKeypointExtractor<PointT>(param));
    keypoint_extractor.push_back(boost::dynamic_pointer_cast<KeypointExtractor<PointT>>(ke));
  }
  if (keypoint_extractor.empty()) {
    std::cerr << "Keypoint extractor method " << method << " is not implemented! " << std::endl;
  }

  return keypoint_extractor;
}
}

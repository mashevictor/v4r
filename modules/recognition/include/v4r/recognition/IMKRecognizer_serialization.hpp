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
 * @file IMKRecognizer_serialization.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KEYPOINT_OBJECT_RECOGNIZER_BOOST_SERIALIZATION
#define KEYPOINT_OBJECT_RECOGNIZER_BOOST_SERIALIZATION

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/base_object.hpp>
#include <v4r/keypoints/impl/pair_serialization.hpp>
#include <v4r/keypoints/impl/eigen_boost_serialization.hpp>
#include <v4r/keypoints/impl/opencv_serialization.hpp>
#include <v4r/keypoints/impl/triple_serialization.hpp>
#include <v4r/recognition/IMKView.h>


 

/** View serialization **/
namespace boost{namespace serialization{

  template<class Archive>
  void serialize(Archive & ar, v4r::IMKView &view, const unsigned int version)
  {
      (void)version;
    ar & view.object_id;
    ar & view.points;
    ar & view.keys;
    ar & view.cloud.type;
    ar & view.cloud.rows;
    ar & view.cloud.cols;
    ar & view.cloud.data;
    ar & view.weight_mask;
    ar & view.conf_desc;
  }

}}


#endif

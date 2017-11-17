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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#ifndef KP_POINT_TYPES_HPP
#define KP_POINT_TYPES_HPP

#include <stdint.h>
#include <stdio.h>
#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

namespace v4r {

const float NaNf = std::numeric_limits<float>::quiet_NaN();

/**
 * PointXYZRGB
 */
class V4R_EXPORTS PointXYZRGB {
 public:
  Eigen::Vector4f pt;
  union {
    struct {
      uint8_t b;
      uint8_t g;
      uint8_t r;
      uint8_t a;
    };
    float rgb;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZRGB() : pt(Eigen::Vector4f(NaNf, NaNf, NaNf, 1.)) {}

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap() {
    return Eigen::Vector3f::Map(&pt[0]);
  }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const {
    return Eigen::Vector3f::Map(&pt[0]);
  }
  inline Eigen::Vector4f &getVector4fMap() {
    return pt;
  }
  inline const Eigen::Vector4f &getVector4fMap() const {
    return pt;
  }

  inline float &operator[](int i) {
    return pt[i];
  }
  inline const float &operator[](int i) const {
    return pt[i];
  }
};

/**
 * PointXYZNormalRGB
 */
class V4R_EXPORTS PointXYZNormalRGB {
 public:
  Eigen::Vector4f pt;
  Eigen::Vector4f n;
  union {
    struct {
      uint8_t b;
      uint8_t g;
      uint8_t r;
      uint8_t a;
    };
    float rgb;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZNormalRGB() : pt(Eigen::Vector4f(NaNf, NaNf, NaNf, 1.)), n(Eigen::Vector4f(NaNf, NaNf, NaNf, 1.)) {}

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap() {
    return Eigen::Vector3f::Map(&pt[0]);
  }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const {
    return Eigen::Vector3f::Map(&pt[0]);
  }
  inline Eigen::Vector4f &getVector4fMap() {
    return pt;
  }
  inline const Eigen::Vector4f &getVector4fMap() const {
    return pt;
  }
  inline Eigen::Map<Eigen::Vector3f> getNormalVector3fMap() {
    return Eigen::Vector3f::Map(&n[0]);
  }
  inline const Eigen::Map<const Eigen::Vector3f> getNormalVector3fMap() const {
    return Eigen::Vector3f::Map(&n[0]);
  }
  inline Eigen::Vector4f &getNormalVector4fMap() {
    return n;
  }
  inline const Eigen::Vector4f &getNormalVector4fMap() const {
    return n;
  }

  inline float &operator[](int i) {
    return pt[i];
  }
  inline const float &operator[](int i) const {
    return pt[i];
  }
};

/**
 * PointXYZ (might be wrong!!!!)
 */
class V4R_EXPORTS PointXYZ {
 public:
  Eigen::Vector3f pt;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZ() : pt(Eigen::Vector3f(NaNf, NaNf, NaNf)) {}

  inline Eigen::Vector3f &getVector3fMap() {
    return pt;
  }
  inline const Eigen::Vector3f &getVector3fMap() const {
    return pt;
  }

  inline float &operator[](int i) {
    return pt[i];
  }
  inline const float &operator[](int i) const {
    return pt[i];
  }
};

}  //--END--

#endif

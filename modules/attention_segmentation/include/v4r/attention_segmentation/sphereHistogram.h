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

#ifndef EPSPHEREHISTOGRAM_HPP
#define EPSPHEREHISTOGRAM_HPP

#include "v4r/attention_segmentation/convertions.h"
#include "v4r/attention_segmentation/eputils_headers.h"

struct v1v2new_v {
  unsigned int v1, v2, new_v;
};

class FacePatch {
 public:
  unsigned int vs[3];  // vertices

  cv::Point3d norm;  // normal
  float weight;      // what ever you want to accumulate

  FacePatch() : weight(0.){};
};

class SphereHistogram {
 public:
  std::vector<cv::Point3d> vertices;
  std::vector<FacePatch> faces;  // 20 icosahedron faces

  SphereHistogram();
  void Subdevide();
  void ComputeNormals();
  int FindMatch(cv::Point3d &n);

 private:
  void InitIcosahedron();
  unsigned int AddMidpoint(unsigned v1, unsigned v2);
  void SubdevideFace(FacePatch &face, std::vector<FacePatch> &newFaces);
  bool findEdge(unsigned int v1, unsigned int v2, unsigned int &new_v);

  std::vector<v1v2new_v> checkedVertices;
};

#endif  // EPSPHEREHISTOGRAM_HPP

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


#ifndef SURFACE_PLANE_HH
#define SURFACE_PLANE_HH

#include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>


namespace surface
{

/**
 * some utils for surface modeling
 */
class Plane
{
private:


public:
  Plane(){};
  ~Plane(){};

  template<typename T1,typename T2, typename T3>
  static inline void Exp2Imp(T1 p1[3], T2 p2[3], T3 p3[3], T1 &a, T1 &b, T1 &c, T1 &d);
  template<typename T1,typename T2>
  static inline T1 ImpPointDist(T1 a, T1 b, T1 c, T1 d, T2 p[3]);
  template<typename T1,typename T2, typename T3>
  static inline T1 NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3]);
  template<typename T1,typename T2, typename T3>
  static inline void Exp2Normal(T1 p1[3], T2 p2[3], T3 p3[3], T1 n[3]);
};




/*********************** INLINE METHODES **************************/

// plane coefficients based on three points
template<typename T1,typename T2, typename T3>
inline void Plane::Exp2Imp(T1 p1[3], T2 p2[3], T3 p3[3], T1 &a, T1 &b, T1 &c, T1 &d)
{
  a = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
    - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  b = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
    - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  c = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
    - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  d = - p2[0] * a - p2[1] * b - p2[2] * c;
}

// distance from the point to the plane
template<typename T1,typename T2>
inline T1 Plane::ImpPointDist(T1 a, T1 b, T1 c, T1 d, T2 p[3])
{
  return fabs ( a * p[0] + b * p[1] + c * p[2] + d ) /
         sqrt ( a * a + b * b + c * c );
}

// distance along the normal
template<typename T1,typename T2, typename T3>
inline T1 Plane::NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3])
{
  T1 t[3];

  t[0] = pd[0] - p[0];
  t[1] = pd[1] - p[1];
  t[2] = pd[2] - p[2];

  return t[0]*n[0] + t[1]*n[1] + t[2]*n[2];
}

// @ep what is the return value? n ? but it is not reference!!!
template<typename T1,typename T2, typename T3>
inline void Plane::Exp2Normal(T1 p1[3], T2 p2[3], T3 p3[3], T1 n[3])
{
  T1 norm;

  n[0] = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
       - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  n[1] = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
       - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  n[2] = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
       - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  norm = sqrt ( pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2) );

// @ep: comparison to zero in float???
  if ( norm == 0.0 ) {
    n[0] = n[1] = n[2] = std::numeric_limits<float>::quiet_NaN();;
  }
  else {
    n[0] /= norm;
    n[1] /= norm;
    n[2] /= norm;
  }

}



}

#endif


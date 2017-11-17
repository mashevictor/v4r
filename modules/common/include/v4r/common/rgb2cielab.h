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
 * @file rgb2cielab.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2013
 * @brief RGB to CIELAB converter
 *
 */
#ifndef V4R_RGB2CIELAB_TRANSFORMS___
#define V4R_RGB2CIELAB_TRANSFORMS___

#include <v4r/common/color_transforms.h>

namespace v4r {

class V4R_EXPORTS RGB2CIELAB : public ColorTransform {
 private:
  std::vector<float> sRGB_LUT;
  std::vector<float> sXYZ_LUT;

  //    static omp_lock_t initialization_lock;
  //    static bool is_initialized_;

  void initializeLUT();

 public:
  typedef boost::shared_ptr<RGB2CIELAB> Ptr;

  RGB2CIELAB() {
    initializeLUT();
  }

  /**
   * @brief Converts RGB color in LAB color space defined by CIE
   * @param R (0...255)
   * @param G (0...255)
   * @param B (0...255)
   * @param L (0...100)
   * @param A (approx -170...100)
   * @param B2 (approx -100...150)
   */
  Eigen::VectorXf do_conversion(unsigned char R, unsigned char G, unsigned char B) const;

  /**
   * @brief Converts RGB color into normalized LAB color space
   * @param R (0...255)
   * @param G (0...255)
   * @param B (0...255)
   * @param L (-1...1)
   * @param A (-1...1)
   * @param B2 (-1...1)
   */
  //    static void
  //    RGB2CIELAB_normalized (unsigned char R, unsigned char G, unsigned char B, float &L, float &A,float &B2);

  void do_inverse_conversion(const Eigen::VectorXf &converted_color, unsigned char &R, unsigned char &G,
                             unsigned char &B) const;

  size_t getOutputNumColorCompenents() const {
    return 3;
  }
};
}
#endif

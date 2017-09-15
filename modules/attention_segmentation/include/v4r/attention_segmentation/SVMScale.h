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
 * @file SVMScale.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Scales features.
 */


#ifndef SVM_SCALE_H
#define SVM_SCALE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

#include <v4r/core/macros.h>

namespace svm
{
  
  
/**
 * @brief Class SVMScale: 
 */
class V4R_EXPORTS SVMScale
{
 
public:
  
private:

  double lower, upper;
  std::string save_filename;
  std::string features_filename;
  std::string features_scaled_filename;
  bool have_save_filename;
  bool have_features_filename;
  bool have_features_scaled_filename;
  
  std::vector<double> feature_max;
  std::vector<double> feature_min;
  
  void scale(int index, double &value);
  
public:
  SVMScale(); 
  ~SVMScale();
  
  void setLower(double _lower) {lower = _lower;};
  void setUpper(double _upper) {upper = _upper;};
  void setSaveFileName(std::string _save_filename) {save_filename = _save_filename; have_save_filename = true;};
  void setFeaturesFileName(std::string _features_filename) {features_filename = _features_filename; have_features_filename = true;};
  void setFeaturesScaledFileName(std::string _features_scaled_filename) {features_scaled_filename = _features_scaled_filename; have_features_scaled_filename = true;};
  
  void compute();
};

}

#endif //SVM_SCALE_H

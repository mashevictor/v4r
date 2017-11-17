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
 * @file SVMPredictorSingle.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#ifndef SVM_PREDICTOR_SINGLE_H
#define SVM_PREDICTOR_SINGLE_H

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include "v4r/attention_segmentation//SurfaceModel.h"
#include "v4r/attention_segmentation/svm.h"

namespace svm {

/**
 * @brief Class SVMPredictorSingle:
 */
class SVMPredictorSingle {
 public:
 private:
  std::string model_filename;

  bool have_model_filename;
  bool have_model_node;
  bool have_relations;
  bool have_type;

  struct svm_model *model;  ///< SVM model

  bool predict_probability;  ///< Predict with probability values
  std::vector<v4r::Relation> relations;
  int type;
  int max_nr_attr;                  ///< Maximum attributes = maximum size of feature vector
  struct svm_node *node;            ///< node of svm
  bool scale;                       ///< set scaling on/off
  double lower, upper;              ///< lower/upper limits
  std::vector<double> feature_max;  ///< maximum feature value for scaling
  std::vector<double> feature_min;  ///< minimum feature value for scaling
  std::vector<bool> feature_scaled;

  void checkSmallPatches(unsigned int max_size);
  void scaleValues(std::vector<double> &val);

  bool have_surfaces;
  std::vector<v4r::SurfaceModel::Ptr> surfaces;  ///< Surfaces

 public:
  SVMPredictorSingle();
  ~SVMPredictorSingle();

  void setPredictProbability(bool _predict_probability) {
    predict_probability = _predict_probability;
  };
  void setModelFilename(std::string _model_filename);
  void setRelations(std::vector<v4r::Relation> _relations);
  /** Set surfaces **/
  void setSurfaces(const std::vector<v4r::SurfaceModel::Ptr> _surfaces);
  void setType(int _type);
  /** Classification of all feature vectors of a view of a specific type (1=neighboring / 2=non-neighboring **/
  void compute();
  /** Set scaling of result vector **/
  void setScaling(bool _scale, std::string filename);

  /** Get modified relations **/
  inline std::vector<v4r::Relation> getRelations();

  double predict(std::vector<double> &val, std::vector<double> &prob);
};

inline std::vector<v4r::Relation> SVMPredictorSingle::getRelations() {
  return relations;
}
}

#endif  // SVM_PREDICTOR_SINGLE_H

#ifndef SVM_TRAIN_MODEL_H
#define SVM_TRAIN_MODEL_H

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
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
 * @file SVMTrainModel.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Trains svm.
 */

#include <v4r/core/macros.h>
#include <cstring>
#include <vector>
#include "v4r/attention_segmentation/svm.h"

namespace svm {

/**
 * @brief Class SVMTrainModel:
 */
class V4R_EXPORTS SVMTrainModel {
 public:
 private:
  struct svm_parameter param;  // set by parse_command_line
  struct svm_problem prob;     // set by read_problem
  struct svm_model *model;
  struct svm_node *x_space;
  int cross_validation;
  int nr_fold;

  char input_file_name[1024];
  char model_file_name[1024];

  bool have_input_file_name;
  bool have_model_file_name;

 public:
  SVMTrainModel();
  ~SVMTrainModel(){};

  void setSVMType(int _svm_type);
  void setKernelType(int _kernel_type);
  void setDegree(int _degree);
  void setGamma(double _gamma);
  void setCoef0(double _coef0);
  void setNu(double _nu);
  void setCacheSize(double _cache_size);
  void setC(double _C);
  void setEps(double _eps);
  void setP(double _p);
  void setShrinking(int _shrinking);
  void setProbability(int _probability);
  void setCrossValidation(int _nr_fold);
  void setWeight(int _i, float _weight);
  void setInputFileName(std::string _input_file_name);
  void setModelFileName(std::string _input_file_name);
  void setNoPrint(bool _no_print);
  int train(double &RecRate, std::vector<int> &ConfusionTable);

 private:
  void readProblem(const char *filename);
  void do_cross_validation(double &RecRate, std::vector<int> &ConfusionTable);
};
}

#endif  // SVM_TRAIN_MODEL_H

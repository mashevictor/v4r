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
 * @file SVMFileCreator.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#ifndef SVM_SVM_FILE_CREATOR_H
#define SVM_SVM_FILE_CREATOR_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "v4r/attention_segmentation//Relation.h"
#include "v4r/attention_segmentation//SurfaceModel.h"

namespace svm
{

/**
 * @brief Class SVMFileCreator: 
 */
class SVMFileCreator
{
public:
  
private:

  bool analyze;
  bool testset;
  std::vector<v4r::Relation> relations;
  bool have_relations;
  std::vector<v4r::SurfaceModel::Ptr> surfaces;
  bool have_surfaces;
  std::string filename_base, filename_base_as;
  int featureNumber;

public:
  SVMFileCreator();
  ~SVMFileCreator();
  
  /** Set input surface patches **/
  void setRelations(std::vector<v4r::Relation> _relations);

  /** Set relations **/
  void setSurfaces(std::vector<v4r::SurfaceModel::Ptr> _surfaces);
  
  /** Print aditional positive/negative file for analysation. **/
  void setAnalyzeOutput(bool _analyze) {analyze = _analyze;}
  
  /** Set true for testset output **/
  void setTestSet(bool _testset) {testset = _testset;}
  
  /** Set true for testset output **/
  void setFilenameBase(std::string _filename_base) {filename_base = _filename_base;}

  /** Set true for testset output **/
  void setFilenameAsBase(std::string _filename_base_as) {filename_base_as = _filename_base_as;}
  
  /** Set true for testset output **/
  void setFeatureNumber(int _featureNumber) {featureNumber = _featureNumber;}
  
  /** Print relations for both levels to file (append) **/
  void process();
};

}

#endif


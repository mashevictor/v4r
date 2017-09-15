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
 * @file IMKRecognizerIO.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef KP_IMK_RECOGNIZER_IO_HH
#define KP_IMK_RECOGNIZER_IO_HH

#include <iostream>
#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <v4r/recognition/IMKRecognizer.h>
#include <v4r/recognition/IMKRecognizer_serialization.hpp>
#include <v4r/keypoints/CodebookMatcher.h>
#include <opencv2/core/core.hpp>
#include "boost/filesystem.hpp"


namespace v4r
{



/*************************************************************************** 
 * IMKRecognizerIO
 */
class V4R_EXPORTS IMKRecognizerIO
{
private:
  static void generateDir(const std::string &dir, const std::vector<std::string> &object_names, std::string &full_dir);
  static void generateName(const std::string &dir, const std::vector<std::string> &object_names, std::string &full_name);

public:
  IMKRecognizerIO() {};

  /** write **/
  static void write(const std::string &dir, const std::vector<std::string> &object_names, const std::vector<IMKView> &object_models, const CodebookMatcher &cb, const std::string &codebookFilename="");

  /** read **/
  static bool read(const std::string &dir, std::vector<std::string> &object_names, std::vector<IMKView> &object_models, CodebookMatcher &cb, const std::string &codebookFilename="");
};





} //--END--

#endif


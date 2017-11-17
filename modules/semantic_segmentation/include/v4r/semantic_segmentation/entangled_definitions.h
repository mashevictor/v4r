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
 * @file   entangled_definitions.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once

#include <array>
#include <vector>

#define LEAF_WEIGHTED
//#define USE_RF_ENERGY
#define COIN_FLIP
#define PI 3.141592654

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29577951308232087721)
#endif

#define LOG_ERROR(...) std::cout << "\033[1;31m" << __VA_ARGS__ << "\033[0m" << std::endl;
#define LOG_INFO(...) std::cout << "\033[1;33m" << __VA_ARGS__ << "\033[0m" << std::endl;
#define LOG_PLAIN(...) std::cout << __VA_ARGS__ << std::endl;

typedef std::array<unsigned int, 3> PointIdx;
typedef std::vector<std::array<unsigned int, 3>> PointIndices;
typedef std::vector<std::array<unsigned int, 3>>::iterator PointIdxItr;

typedef std::array<int, 2> ClusterIdx;  // first value is image idx, second value cluster idx
typedef std::vector<ClusterIdx> ClusterIndices;
typedef ClusterIndices::iterator ClusterIdxItr;

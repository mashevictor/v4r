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
 * @file histogram.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */


#include <v4r/core/macros.h>
#include <Eigen/Core>
#ifndef V4R_COMMON_HISTOGRAM_H_
#define V4R_COMMON_HISTOGRAM_H_

namespace v4r
{

/**
 * @brief compute histogram of the row entries of a matrix
 * @param[in] data (columns are the elements, rows are the different dimensions
 * @param[out] histogram
 * @param[in] number of bins
 * @param[in] range minimum
 * @param[in] range maximum
 */
V4R_EXPORTS void
computeHistogram (const Eigen::MatrixXf &data, Eigen::MatrixXi &histogram, size_t bins=100, float min=0.f, float max=1.f);



/**
 * @brief compute cumulative histogram
 * @param[in] histogram
 * @param[out] cumulative histogram
 */
V4R_EXPORTS void
computeCumulativeHistogram (const Eigen::VectorXi &histogram, Eigen::VectorXi &cumulative_histogram);



/**
 * @brief computes histogram intersection (does not normalize histograms!)
 * @param[in] histA
 * @param[in] histB
 * @return intersection value
 */
V4R_EXPORTS int
computeHistogramIntersection (const Eigen::VectorXi &histA, const Eigen::VectorXi &histB);



/**
 * @brief shift histogram values by one bin
 * @param[in] hist
 * @param[out] hist_shifted
 * @param[in] direction_is_right (if true, shift histogram to the right. Otherwise to the left)
 */
V4R_EXPORTS void
shiftHistogram (const Eigen::VectorXi &hist, Eigen::VectorXi &hist_shifted, bool direction_is_right=true);



/**
 * @brief specifyHistogram (based on http://fourier.eng.hmc.edu/e161/lectures/contrast_transform/node3.html)
 * @param input_image color values of input image
 * @param desired_color color values of desired image
 * @param bins histogram bins
 * @param min minimum color value
 * @param max maximum color value
 * @return specified histogram
 */
V4R_EXPORTS Eigen::VectorXf
specifyHistogram (const Eigen::VectorXf &input_image, const Eigen::VectorXf &desired_image, size_t bins=100, float min=0.f, float max=1.f);

}

#endif

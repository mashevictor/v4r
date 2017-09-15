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
 * @file ourcvfh_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2012
 * @brief OURCVFH Estimator
 *
 */
#pragma once

#include <v4r/core/macros.h>
#include <v4r/features/global_estimator.h>
#include <v4r/features/types.h>

namespace v4r
{
class V4R_EXPORTS OURCVFHEstimatorParameter
{
public:
    std::vector<float> eps_angle_threshold_vector_;
    std::vector<float> curvature_threshold_vector_;
    std::vector<float> cluster_tolerance_vector_;
    float refine_factor_;
    bool normalize_bins_;
    size_t min_points_;
    float axis_ratio_;
    float min_axis_value_;


    OURCVFHEstimatorParameter() :
        eps_angle_threshold_vector_ ( { 10.f*M_PI/180.f } ),
        curvature_threshold_vector_ ( {0.04} ),
        cluster_tolerance_vector_ ( {0.02f} ), //3.f, 0.015f
        refine_factor_ (1.f),
        normalize_bins_ (false),
        min_points_(50),
        axis_ratio_ (0.8f),
        min_axis_value_(0.925f)
    {}
};

template<typename PointT>
class V4R_EXPORTS OURCVFHEstimator : public GlobalEstimator<PointT>
{
private:
    using GlobalEstimator<PointT>::indices_;
    using GlobalEstimator<PointT>::cloud_;
    using GlobalEstimator<PointT>::normals_;
    using GlobalEstimator<PointT>::descr_name_;
    using GlobalEstimator<PointT>::descr_type_;
    using GlobalEstimator<PointT>::feature_dimensions_;
    using GlobalEstimator<PointT>::transforms_;

    OURCVFHEstimatorParameter param_;

public:
    OURCVFHEstimator(const OURCVFHEstimatorParameter &p = OURCVFHEstimatorParameter() )
        :
          GlobalEstimator<PointT>("ourcvfh", FeatureType::OURCVFH, 308),
          param_(p)
    { }

    bool
    compute (Eigen::MatrixXf &signature);

    bool
    needNormals() const
    {
        return true;
    }
};
}

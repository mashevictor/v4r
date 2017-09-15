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
 * @file classifier.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <v4r/ml/types.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

namespace v4r
{
class V4R_EXPORTS Classifier
{
public:
    Classifier()
    {}

    /**
     * @brief train the classifer
     * @param training_data (each training data point is a row entry, the feature dimensions are equal to the number of columns)
     * @param training_label (the label for each training data point)
     */
    virtual void
    train( const Eigen::MatrixXf &training_data, const Eigen::VectorXi & training_label) = 0;

    /**
     * @brief predict the target value of a query feature
     * @param query_data (each query is a row entry, the feature dimensions are equal to the number of columns)
     * @param predicted_label (each query produces a row of predicted labels, the columns of the predicted labels correspond to the most probable predictions. Predictions are sorted - most likely one is on the left)
     */
    virtual void
    predict(const Eigen::MatrixXf &query_data, Eigen::MatrixXi &predicted_label) const = 0;

    virtual void
    getTrainingSampleIDSforPredictions(Eigen::MatrixXi &predicted_training_sample_indices, Eigen::MatrixXf &distances)
    {
        (void)predicted_training_sample_indices;
        (void)distances;
        std::cerr << "getTrainingSampleIDSforPredictions is not implemented right now." << std::endl;
    }

    virtual int
    getType() const = 0;

    typedef boost::shared_ptr< Classifier > Ptr;
    typedef boost::shared_ptr< Classifier const> ConstPtr;
};

}

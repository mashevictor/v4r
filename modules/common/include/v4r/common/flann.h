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
 * @file flann.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2015
 * @brief  FLANN helper functions for conversion from Eigen (useful for e.g. fast nearest neighbor search in high-dimensional space)
 *
 */

#ifndef V4R_FLANN_H__
#define V4R_FLANN_H__

#include <pcl/kdtree/flann.h>
#include <v4r/core/macros.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>

namespace v4r
{

class V4R_EXPORTS EigenFLANN
{
public:
    typedef boost::shared_ptr< EigenFLANN > Ptr;
    typedef boost::shared_ptr< EigenFLANN const> ConstPtr;

    class V4R_EXPORTS Parameter
    {
    public:
        int kdtree_splits_;
        int distance_metric_; ///< defines the norm used for feature matching (1... L1 norm, 2... L2 norm)
        int knn_;

        Parameter(
                int kdtree_splits = 128,
                int distance_metric = 2,
                int knn = 1
                )
            : kdtree_splits_ (kdtree_splits),
              distance_metric_ (distance_metric),
              knn_ (knn)
        {}
    }param_;

private:
    boost::shared_ptr< typename flann::Index<flann::L1<float> > > flann_index_l1_;
    boost::shared_ptr< typename flann::Index<flann::L2<float> > > flann_index_l2_;
    boost::shared_ptr<flann::Matrix<float> > flann_data_;

public:
    EigenFLANN(const Parameter &p = Parameter()) : param_(p) { }

    /**
     * @brief creates a FLANN index
     * @param signatures matrix with size num_features x dimensionality
     * @return
     */
    bool
    createFLANN ( const Eigen::MatrixXf &data);


    /**
     * @brief nearestKSearch perform nearest neighbor search for the rows queries in query_signature
     * @param query_signature (rows = num queries; cols = feature dimension)
     * @param indices (rows = num queries; cols = nearest neighbor indices)
     * @param distances (rows = num queries; cols = nearest neighbor distances)
     * @return
     */
    bool
    nearestKSearch (const Eigen::MatrixXf &query_signature, Eigen::MatrixXi &indices, Eigen::MatrixXf &distances) const;
};
}

#endif

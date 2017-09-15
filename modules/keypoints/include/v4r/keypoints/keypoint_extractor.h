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
 * @file keypoint_extractor.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2013
 * @brief
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <pcl/common/common.h>
#include <v4r/keypoints/types.h>

namespace v4r
{

template<typename PointT>
class V4R_EXPORTS KeypointExtractor
{
protected:
    typename pcl::PointCloud<PointT>::ConstPtr input_; ///< input cloud
    pcl::PointCloud<pcl::Normal>::ConstPtr normals_; ///< surface normals for input cloud
    typename pcl::PointCloud<PointT>::Ptr keypoints_; /// extracted keypoints
    std::vector<int> keypoint_indices_; ///< extracted keypoint indices
    std::vector<int> indices_;  ///< indices of the segmented object (extracted keypoints outside of this will be neglected)

public:
    virtual ~KeypointExtractor() = 0;

    /**
     * @brief setInputCloud
     * @param input input cloud
     */
    void
    setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr & input)
    {
        input_ = input;
    }

    void
    setNormals (const pcl::PointCloud<pcl::Normal>::ConstPtr & normals)
    {
        normals_ = normals;
    }

    virtual bool
    needNormals () const
    {
        return false;
    }

    std::vector<int>
    getKeypointIndices () const
    {
        return keypoint_indices_;
    }

    /**
     * @brief setIndices
     * @param indices indices of the segmented object (extracted keypoints outside of this will be neglected)
     */
    void
    setIndices(const std::vector<int> &indices)
    {
        indices_ = indices;
    }

    /**
     * @brief getKeypointExtractorType
     * @return unique type id of keypoint extractor (as stated in keypoint/types.h)
     */
    virtual int getKeypointExtractorType() const = 0;

    /**
     * @brief getKeypointExtractorName
     * @return type name of keypoint extractor
     */
    virtual std::string getKeypointExtractorName() const = 0;

    /**
     * @brief compute
     * @param keypoints
     */
    virtual void
    compute () = 0;

    /**
     * @brief getKeypoints
     * @return extracted keypoints
     */
    virtual
    typename pcl::PointCloud<PointT>::Ptr
    getKeypoints() const
    {
        return keypoints_;
    }


    typedef boost::shared_ptr< KeypointExtractor<PointT> > Ptr;
    typedef boost::shared_ptr< KeypointExtractor<PointT> const> ConstPtr;
};
}


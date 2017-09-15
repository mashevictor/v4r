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
 * @file occlusion_reasoning.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef V4R_OCCLUSION_REASONING_H_
#define V4R_OCCLUSION_REASONING_H_

#include <boost/dynamic_bitset.hpp>
#include <pcl/point_cloud.h>
#include <v4r/common/camera.h>
#include <v4r/core/macros.h>


namespace v4r
{

/**
 * @brief Class to reason about occlusion
 * @author: Thomas Faeulhammer
 * @date August 2016
 */
template<typename PointTA, typename PointTB>
class V4R_EXPORTS OcclusionReasoner
{
private:
    typename pcl::PointCloud<PointTA>::ConstPtr occluder_cloud_; ///< organized_cloud point cloud that potentially causes occlusion
    typename pcl::PointCloud<PointTB>::ConstPtr cloud_to_be_filtered_; ///< to_be_filtered point cloud to be checked for occlusion
    float occlusion_threshold_m_;   ///< occlusion threshold in meter
    Camera::ConstPtr cam_; ///@brief camera parameters for re-projection to image plane by depth buffering (only used if point clouds are not organized)
    boost::dynamic_bitset<> px_is_visible_; ///< indicates if a pixel re-projected by the cloud_to_be_filtered_ is in front of the occlusion cloud (i.e. if the pixel belongs to the object)

public:
    OcclusionReasoner()
        : occlusion_threshold_m_ (0.01f)
    { }

    /**
     * @brief setCamera
     * @param cam
     */
    void
    setCamera ( const Camera::ConstPtr cam )
    {
        cam_ = cam;
    }

    /**
     * @brief setOcclusionThreshold
     * @param occlusion_thresh_m
     */
    void
    setOcclusionThreshold(float occlusion_thresh_m)
    {
        occlusion_threshold_m_ = occlusion_thresh_m;
    }

    /**
     * @brief setOcclusionCloud
     * @param occlusion_cloud cloud that can cause occlusion
     */
    void
    setOcclusionCloud( const typename pcl::PointCloud<PointTA>::ConstPtr occlusion_cloud )
    {
        occluder_cloud_ = occlusion_cloud;
    }

    /**
     * @brief setInputCloud
     * @param cloud_to_be_filtered object cloud that is checked for occlusion
     */
    void
    setInputCloud( const typename pcl::PointCloud<PointTB>::ConstPtr cloud_to_be_filtered )
    {
        cloud_to_be_filtered_ = cloud_to_be_filtered;
    }

    /**
     * @brief getPixelMask
     * @return indicates if a pixel re-projected by the cloud_to_be_filtered_ is in front of the occlusion cloud (i.e. if the pixel belongs to the object) - bitset size is equal to number of pixel
     */
    boost::dynamic_bitset<>
    getPixelMask() const
    {
        return px_is_visible_;
    }

    /**
     * @brief compute occlusion
     * @return binary mask which indicates for each point of the cloud_to_be_filtered_ if it is visible (true) or occluded (false) - bitset size is equal to number of points of cloud
     */
    boost::dynamic_bitset<> computeVisiblePoints();
};
}

#endif

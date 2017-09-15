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
 * @file model_view.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */

#ifndef V4R_OBJECT_MODELLING_MODELVIEW_H__
#define V4R_OBJECT_MODELLING_MODELVIEW_H__

#include <boost/dynamic_bitset.hpp>
#include <pcl/common/common.h>
#include <v4r/core/macros.h>
#include <v4r/keypoints/ClusterNormalsToPlanes.h>

namespace v4r
{
    namespace object_modelling
    {
        /**
        * @brief This class represents a training view of the object model used for learning
        * @author Thomas Faeulhammer
        * @date July 2015
        * */
        class V4R_EXPORTS modelView
        {
        public:
            class SuperPlane : public ClusterNormalsToPlanes::Plane
            {
            public:
                std::vector<size_t> visible_indices;
                std::vector<size_t> object_indices;
                std::vector<size_t> within_chop_z_indices;
                bool is_filtered;
                SuperPlane() : ClusterNormalsToPlanes::Plane()
                {
                    is_filtered = false;
                }
            };

        public:
            typedef pcl::PointXYZRGB PointT;
            typedef pcl::Histogram<128> FeatureT;

            pcl::PointCloud<PointT>::Ptr  cloud_;
            pcl::PointCloud<pcl::Normal>::Ptr  normal_;
            pcl::PointCloud<PointT>::Ptr  transferred_cluster_;

            std::vector<std::vector<float> >  sift_signatures_;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  supervoxel_cloud_;
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  supervoxel_cloud_organized_;

            std::vector<SuperPlane> planes_;

            std::vector< size_t > scene_points_;
            std::vector< size_t > sift_keypoint_indices_;

            std::vector< boost::dynamic_bitset<> > obj_mask_step_;
            Eigen::Matrix4f camera_pose_;
            Eigen::Matrix4f tracking_pose_;
            bool tracking_pose_set_ = false;
            bool camera_pose_set_ = false;

            size_t id_; // might be redundant

            bool is_pre_labelled_;

            modelView() : is_pre_labelled_(false)
            {
                cloud_.reset(new pcl::PointCloud<PointT>());
                normal_.reset(new pcl::PointCloud<pcl::Normal>());
                transferred_cluster_.reset(new pcl::PointCloud<PointT>());
                supervoxel_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
                supervoxel_cloud_organized_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
            }
        };
    }
}


#endif //V4R_OBJECT_MODELLING_MODELVIEW_H__

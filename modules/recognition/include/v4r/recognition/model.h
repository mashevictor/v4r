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
 * @file model.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */


#pragma once

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/mpl/at.hpp>
#include <boost/mpl/map.hpp>
#include <boost/serialization/vector.hpp>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <v4r/core/macros.h>

namespace v4r
{


/**
 * @brief class to describe a training view of the object model
 * @author Thomas Faeulhammer
 * @date Oct 2016
 */
template<typename PointT>
class V4R_EXPORTS TrainingView
{
private:
    friend class boost::serialization::access;

    template<class Archive> void serialize(Archive & ar, const unsigned int version)
    {
        ar & cloud_;
        ar & pose_;
        ar & filename_;
        ar & pose_filename_;
        ar & indices_filename_;
        ar & indices_;
        ar & view_centroid_;
    }

public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrainingView()
        : pose_(Eigen::Matrix4f::Identity()),
          filename_(""),
          pose_filename_(""),
          indices_filename_(""),
          view_centroid_(Eigen::Vector3f::Zero()),
          self_occlusion_(0.f)
    {}

    typename pcl::PointCloud<PointT>::ConstPtr cloud_; ///< point cloud of view
    typename pcl::PointCloud<pcl::Normal>::ConstPtr normals_; ///< normals for point cloud of view
    Eigen::Matrix4f pose_; ///< corresponding camera pose (s.t. multiplying the individual clouds with these transforms bring it into a common coordinate system)
    std::string filename_; ///< cloud filename of the training view
    std::string pose_filename_; ///< pose filename of the training view
    std::string indices_filename_; ///< object mask/indices filename of the training view
    std::vector<int> indices_; ///< corresponding object indices
    Eigen::Vector3f view_centroid_;  ///< centre of gravity for the 2.5D view of the model
    float self_occlusion_; ///< self-occlusion of respective view
    Eigen::Vector3f elongation_; ///< elongations in meter for each dimension
    Eigen::Matrix4f eigen_pose_alignment_;

    typedef boost::shared_ptr< TrainingView<PointT> > Ptr;
    typedef boost::shared_ptr< TrainingView<PointT> const> ConstPtr;
};

/**
 * @brief Class representing a recognition model
 */
template<typename PointT>
class V4R_EXPORTS Model
{
private:
    mutable pcl::visualization::PCLVisualizer::Ptr vis_;
    mutable int vp1_;

    friend class boost::serialization::access;

    template<class Archive> V4R_EXPORTS void serialize(Archive & ar, const unsigned int version)
    {
        ar & class_;
        ar & id_;
        if (normals_assembled_) ar & *normals_assembled_;
        ar & centroid_;
        ar & centroid_computed_;
        ar & flip_normals_based_on_vp_;
    }

    typedef boost::mpl::map
    <
    boost::mpl::pair<pcl::PointXYZ,          pcl::PointNormal>,
    boost::mpl::pair<pcl::PointNormal,       pcl::PointNormal>,
    boost::mpl::pair<pcl::PointXYZRGB,       pcl::PointXYZRGBNormal>,
    boost::mpl::pair<pcl::PointXYZRGBA,      pcl::PointXYZRGBNormal>,
    boost::mpl::pair<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>,
    boost::mpl::pair<pcl::PointXYZI,         pcl::PointXYZINormal>,
    boost::mpl::pair<pcl::PointXYZINormal,   pcl::PointXYZINormal>
    > PointTypeAssociations;
    BOOST_MPL_ASSERT ((boost::mpl::has_key<PointTypeAssociations, PointT>));

    typedef typename boost::mpl::at<PointTypeAssociations, PointT>::type PointTWithNormal;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
    typedef typename pcl::PointCloud<PointT>::ConstPtr PointTPtrConst;
    std::vector<typename TrainingView<PointT>::ConstPtr> views_;
    std::string class_, id_;
    Eigen::Vector4f minPoint_; ///< defines the 3D bounding box of the object model
    Eigen::Vector4f maxPoint_; ///< defines the 3D bounding box of the object model
    PointTPtr assembled_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_assembled_;
    mutable typename std::map<int, PointTPtrConst> voxelized_assembled_;
    mutable typename std::map<int, pcl::PointCloud<pcl::Normal>::ConstPtr> normals_voxelized_assembled_;
    Eigen::Vector4f centroid_;    ///< centre of gravity for the whole 3d model
    bool centroid_computed_;

    pcl::PointCloud<pcl::PointXYZL>::Ptr faces_cloud_labels_;
    typename std::map<int, pcl::PointCloud<pcl::PointXYZL>::Ptr> voxelized_assembled_labels_;
    bool flip_normals_based_on_vp_;

    Model() : centroid_computed_ (false), flip_normals_based_on_vp_ (false)
    { }

    bool getFlipNormalsBasedOnVP() const
    {
        return flip_normals_based_on_vp_;
    }

    void setFlipNormalsBasedOnVP(bool b)
    {
        flip_normals_based_on_vp_ = b;
    }

    bool
    operator== (const Model &other) const
    {
        return (id_ == other.id_) && (class_ == other.class_);
    }

    /**
     * @brief addTrainingView
     * @param tv training view
     */
    void
    addTrainingView(const typename TrainingView<PointT>::ConstPtr &tv)
    {
        views_.push_back( tv );
    }

    std::vector<typename TrainingView<PointT>::ConstPtr >
    getTrainingViews() const
    {
        return views_;
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr getAssembledSmoothFaces (int resolution_mm);

    typename pcl::PointCloud<PointT>::ConstPtr getAssembled(int resolution_mm) const;

    /**
     * @brief initialize initializes the model creating 3D models and so on
     */
    void
    initialize(const std::string &model_filename = "");

    pcl::PointCloud<pcl::Normal>::ConstPtr getNormalsAssembled (int resolution_mm) const;

    typedef boost::shared_ptr< Model<PointT> > Ptr;
    typedef boost::shared_ptr< Model<PointT> const> ConstPtr;
};

}

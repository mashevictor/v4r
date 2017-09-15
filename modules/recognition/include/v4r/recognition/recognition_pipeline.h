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
 * @file recognition_pipeline.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <pcl/common/common.h>

#include <v4r_config.h>
#include <v4r/common/normals.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/core/macros.h>
#include <v4r/recognition/object_hypothesis.h>
#include <v4r/recognition/source.h>
#include <glog/logging.h>

namespace v4r
{

/**
 * @brief The recognition pipeline class is an abstract class that represents a
 * pipeline for object recognition. It will generated groups of object hypotheses.
 * For a global recognition pipeline, each segmented cluster from the input cloud will store its object hypotheses into one group.
 * For all other pipelines, each group will only contain one object hypothesis.
 * @author Thomas Faeulhammer, Aitor Aldoma
 */
template<typename PointT>
class V4R_EXPORTS RecognitionPipeline
{
public:
    typedef boost::shared_ptr< RecognitionPipeline<PointT> > Ptr;
    typedef boost::shared_ptr< RecognitionPipeline<PointT> const> ConstPtr;

protected:
    typedef Model<PointT> ModelT;
    typedef boost::shared_ptr<ModelT> ModelTPtr;

    typename pcl::PointCloud<PointT>::ConstPtr scene_; ///< Point cloud to be recognized
    pcl::PointCloud<pcl::Normal>::ConstPtr scene_normals_; ///< associated normals
    typename Source<PointT>::ConstPtr m_db_;  ///< model data base
    std::vector< ObjectHypothesesGroup > obj_hypotheses_;   ///< generated object hypotheses
    typename NormalEstimator<PointT>::Ptr normal_estimator_;    ///< normal estimator used for computing surface normals (currently only used at training)
    Eigen::Vector4f table_plane_;
    bool table_plane_set_;

    static std::vector< std::pair<std::string,float> > elapsed_time_;  ///< to measure performance

    class StopWatch
    {
        std::string desc_;
        boost::posix_time::ptime start_time_;

    public:
        StopWatch(const std::string &desc)
            :desc_ (desc), start_time_ (boost::posix_time::microsec_clock::local_time ())
        {}

        ~StopWatch()
        {
            boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
            float elapsed_time = static_cast<float> (((end_time - start_time_).total_milliseconds ()));
            VLOG(1) << desc_ << " took " << elapsed_time << " ms.";
            elapsed_time_.push_back( std::pair<std::string,float>(desc_, elapsed_time) );
        }
    };


    PCLVisualizationParams::ConstPtr vis_param_;

public:
    RecognitionPipeline() :
        table_plane_(Eigen::Vector4f::Identity()),
        table_plane_set_(false)
    {}

    virtual ~RecognitionPipeline(){}

    virtual size_t getFeatureType() const = 0;

    virtual bool needNormals() const = 0;

    /**
     * @brief initialize the recognizer (extract features, create FLANN,...)
     * @param[in] path to model database. If training directory exists, will load trained model from disk; if not, computed features will be stored on disk (in each
     * object model folder, a feature folder is created with data)
     * @param[in] retrain if set, will re-compute features and store to disk, no matter if they already exist or not
     */
    virtual void
    initialize(const std::string &trained_dir = "", bool retrain = false)
    {
        (void) retrain;
        (void) trained_dir;
        PCL_WARN("initialize is not implemented for this class.");
    }

    /**
     * @brief setInputCloud
     * @param cloud to be recognized
     */
    void
    setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr cloud)
    {
        scene_ = cloud;
    }


    /**
     * @brief getObjectHypothesis
     * @return generated object hypothesis
     */
    std::vector<ObjectHypothesesGroup>
    getObjectHypothesis() const
    {
        return obj_hypotheses_;
    }

    /**
     * @brief setSceneNormals
     * @param normals normals of the input cloud
     */
    void
    setSceneNormals(const pcl::PointCloud<pcl::Normal>::ConstPtr &normals)
    {
        scene_normals_ = normals;
    }

    /**
     * @brief setModelDatabase
     * @param m_db model database
     */
    void
    setModelDatabase(const typename Source<PointT>::ConstPtr &m_db)
    {
        m_db_ = m_db;
    }

    /**
     * @brief getModelDatabase
     * @return model database
     */
    typename Source<PointT>::ConstPtr
    getModelDatabase() const
    {
        return m_db_;
    }

    void
    setTablePlane( const Eigen::Vector4f &table_plane)
    {
        table_plane_ = table_plane;
        table_plane_set_ = true;
    }


    /**
     * @brief setNormalEstimator sets the normal estimator used for computing surface normals (currently only used at training)
     * @param normal_estimator
     */
    void
    setNormalEstimator(const typename NormalEstimator<PointT>::Ptr &normal_estimator)
    {
        normal_estimator_ = normal_estimator;
    }


    /**
     * @brief setVisualizationParameter sets the PCL visualization parameter (only used if some visualization is enabled)
     * @param vis_param
     */
    void
    setVisualizationParameter(const PCLVisualizationParams::ConstPtr &vis_param)
    {
        vis_param_ = vis_param;
    }

    /**
     * @brief getElapsedTimes
     * @return compuation time measurements for various components
     */
    std::vector<std::pair<std::string, float> >
    getElapsedTimes() const
    {
        return elapsed_time_;
    }

    virtual bool requiresSegmentation() const = 0;
    virtual void do_recognize () = 0;

    void
    recognize ()
    {
        elapsed_time_.clear();
        obj_hypotheses_.clear();
        CHECK ( scene_ ) << "Input scene is not set!";

        if( needNormals() )
            CHECK ( scene_normals_ && scene_->points.size() == scene_normals_->points.size()) << "Recognizer needs normals but they are not set!";

        do_recognize();
    }
};
}

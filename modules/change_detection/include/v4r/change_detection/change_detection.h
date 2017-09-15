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
 * @file change_detection.h
 * @author Martin Velas, Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date Aug 2015
 * @brief
 *
 */


#pragma once

#include <v4r/core/macros.h>

#include <pcl/common/eigen.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace v4r
{

class V4R_EXPORTS ChangeDetectorParameters
{
public:
    int occlusion_checker_bins;
    int min_cluster_points;
    int max_cluster_points;
    float maximal_intra_cluster_dist;
    float planarity_threshold;
    float min_removal_overlap;
    float cloud_difference_tolerance;

    ChangeDetectorParameters() :
            occlusion_checker_bins(180),
            min_cluster_points(50),
            max_cluster_points(1000000),
            maximal_intra_cluster_dist(0.03f),
            planarity_threshold(0.9f),
            min_removal_overlap(0.8f),
            cloud_difference_tolerance(0.01f)
    { }
};

template<class PointT>
class V4R_EXPORTS ChangeDetector
{

public:
    typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;
    typedef pcl::search::KdTree<PointT> Tree;

    ChangeDetector() :
        added(new pcl::PointCloud<PointT>()), removed(new pcl::PointCloud<PointT>)
    { }

    void detect(const typename pcl::PointCloud<PointT>::ConstPtr &source, const CloudPtr target, const Eigen::Affine3f sensor_pose,
			float diff_tolerance = DEFAULT_PARAMETERS.cloud_difference_tolerance);

	bool isObjectRemoved(CloudPtr object_cloud) const;

    static float computePlanarity(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud);

    std::vector<typename pcl::PointCloud<PointT>::Ptr>
	static clusterPointCloud(CloudPtr input_cloud, double tolerance,
			int min_cluster_size, int max_cluster_size);

    static bool hasPointInRadius(const PointT &pt, const Tree &tree, float distance)
    {
        if ( !pcl::isFinite(pt))
            return false;

        // We're interested in a single neighbor only
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        return (tree.radiusSearch(pt, distance, nn_indices, nn_distances, 1) >= 1);
    }

	/**
	 * diff = A \ B
	 * indices = indexes of preserved points from A
	 */
    static void difference(
            const pcl::PointCloud<PointT> &A,
            const typename pcl::PointCloud<PointT>::ConstPtr &B,
            pcl::PointCloud<PointT> &diff,
            std::vector<int> &indices,
            float tolerance = DEFAULT_PARAMETERS.cloud_difference_tolerance)
    {
        if(A.empty())
            return;

        if(B->empty())
        {
            pcl::copyPointCloud(A, diff);
            indices.reserve(A.size());
            for(size_t i = 0; i < A.size(); i++)
                indices.push_back(i);

            return;
        }

        typename Tree::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(B);

        // Iterate through the source data set
        indices.resize( A.points.size() );
        size_t kept=0;
        for (size_t i = 0; i < A.points.size(); i++)
        {
            if ( pcl::isFinite( A.points[i] ) && !hasPointInRadius(A.points[i], *tree, tolerance))
                indices[kept++] = i;
        }
        indices.resize(kept);

        diff.points.resize( indices.size() );
        diff.header = A.header;
        diff.width = indices.size();
        diff.height = 1;
        diff.is_dense = true;
        pcl::copyPointCloud(A, indices, diff);
    }

    static void difference(
            const pcl::PointCloud<PointT> &A,
            const typename pcl::PointCloud<PointT>::ConstPtr &B,
            pcl::PointCloud<PointT> &diff)
    {
		std::vector<int> indices;
        difference(A, B, diff, indices);
	}

	static void removePointsFrom(const CloudPtr cloud, const CloudPtr toBeRemoved);

	static int overlapingPoints(const CloudPtr train, const CloudPtr query,
			float tolerance = DEFAULT_PARAMETERS.cloud_difference_tolerance);

    const typename pcl::PointCloud<PointT>::Ptr
    getAdded() const
    {
		return added;
	}

    const typename pcl::PointCloud<PointT>::Ptr
    getRemoved() const
    {
		return removed;
	}

    static CloudPtr getNonplanarClusters(CloudPtr removed_points)
    {
		std::vector<CloudPtr> clusters = clusterPointCloud(removed_points,
				DEFAULT_PARAMETERS.maximal_intra_cluster_dist, DEFAULT_PARAMETERS.min_cluster_points,
				DEFAULT_PARAMETERS.max_cluster_points);
        CloudPtr nonplanarClusters(new pcl::PointCloud<PointT>());
		for(typename  std::vector<CloudPtr>::iterator c = clusters.begin();
				c < clusters.end(); c++) {
			if(computePlanarity(*c) < DEFAULT_PARAMETERS.planarity_threshold) {
				*nonplanarClusters += **c;
			}
		}
		return nonplanarClusters;
	}

	static CloudPtr removalSupport(CloudPtr removed_points, CloudPtr &object_cloud,
			float tolerance = DEFAULT_PARAMETERS.cloud_difference_tolerance,
            float planarity_threshold = DEFAULT_PARAMETERS.planarity_threshold)
    {
        CloudPtr support(new pcl::PointCloud<PointT>());
		std::vector<CloudPtr> removed_clusters = clusterPointCloud(removed_points,
				DEFAULT_PARAMETERS.maximal_intra_cluster_dist, DEFAULT_PARAMETERS.min_cluster_points,
				DEFAULT_PARAMETERS.max_cluster_points);
        for(typename  std::vector<CloudPtr>::iterator c = removed_clusters.begin(); c < removed_clusters.end(); c++) {

			// only non planar cluster considered:
            if(planarity_threshold > 1.f || computePlanarity(*c) < planarity_threshold) {
				if(overlapingPoints(object_cloud, *c, tolerance) > (DEFAULT_PARAMETERS.min_removal_overlap * (*c)->size())) {
					*support += **c;
				}
			}
		}
		return support;
	}

protected:
    typename pcl::PointCloud<PointT>::Ptr removalSupport(CloudPtr &object_cloud) const;

private:
    typename pcl::PointCloud<PointT>::Ptr added, removed;

public:
    static const ChangeDetectorParameters DEFAULT_PARAMETERS;
    ChangeDetectorParameters params;
};

}

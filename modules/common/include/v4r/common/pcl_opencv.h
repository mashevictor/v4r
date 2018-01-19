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
 * @file pcl_opencv.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief helper functions to convert PCL pointclouds to opencv images
 *
 */
#include <pcl/common/common.h>
#include <v4r/common/camera.h>
#include <v4r/core/macros.h>
#include <boost/dynamic_bitset.hpp>
#include <opencv2/opencv.hpp>

#pragma once

namespace v4r {

///
/// \brief The PCLOpenCVConverter class converts PCL point clouds into RGB, depth or occupancy images.
/// \author Thomas Faeulhammer
/// \date August 2016
///
template <typename PointT>
class V4R_EXPORTS PCLOpenCVConverter {
 private:
  typedef std::vector<uchar> (PCLOpenCVConverter<PointT>::*pf)(int v, int u) const;

  typename pcl::PointCloud<PointT>::ConstPtr cloud_;  ///< cloud to be converted
  std::vector<int> indices_;  ///< pixel indices to be extracted (if empty, all pixel will be extracted)
  Camera::ConstPtr cam_;      ///< camera parameters (used for re-projection if point cloud is not organized)
  bool remove_background_;  ///< if true, will set pixel not specified by the indices to the background color defined by
                            /// its member variable background_color_
  cv::Vec3b background_color_;  ///< background color (only used if indices are not empty and remove_background is set
                                /// to true)
  float min_depth_m_;           ///< minimum depth in meter for normalization
  float max_depth_m_;           ///< maximum depth in meter for normalization
  cv::Rect roi_;                ///< region of interest with all given indices taken into account
  cv::Mat output_matrix_;       /// <
  Eigen::MatrixXi index_map_;   ///< index map showing which point of the unorganized(!) point cloud maps to which pixel
                                /// in the image plane (pixel not occupied by any point have value -1)

  cv::Rect computeROIfromIndices();
  void computeOrganizedCloud();

  std::vector<uchar> getRGB(int v, int u) {
    const PointT &pt = cloud_->at(u, v);
    std::vector<uchar> rgb = {pt.b, pt.g, pt.r};
    return rgb;
  }

  std::vector<uchar> getZNormalized(int v, int u) {
    const PointT &pt = cloud_->at(u, v);
    std::vector<uchar> rgb = {
        std::min<uchar>(255, std::max<uchar>(0, 255.f * (pt.z - min_depth_m_) / (max_depth_m_ - min_depth_m_)))};
    return rgb;
  }

  std::vector<float> getZ(int v, int u) {
    const PointT &pt = cloud_->at(u, v);
    float depth;
    pcl_isfinite(pt.z) ? depth = pt.z : depth = 0.f;
    std::vector<float> z = {depth};
    return z;
  }

  std::vector<uchar> getOccupied(int v, int u) {
    const PointT &pt = cloud_->at(u, v);
    if (std::isfinite(pt.z))
      return std::vector<uchar>(1, 255);
    else
      return std::vector<uchar>(1, 0);
  }

 public:
  PCLOpenCVConverter(const typename pcl::PointCloud<PointT>::ConstPtr cloud = nullptr)
  : cloud_(cloud), remove_background_(true), background_color_(cv::Vec3b(255, 255, 255)), min_depth_m_(0.f),
    max_depth_m_(5.f) {}

  ///
  /// \brief setInputCloud
  /// \param cloud point cloud to be converted
  ///
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr cloud) {
    cloud_ = cloud;
    index_map_.resize(0, 0);
  }

  ///
  /// \brief setIndices
  /// \param indices indices to be extracted (if empty, all pixel will be extracted)
  ///
  void setIndices(const std::vector<int> &indices) {
    indices_ = indices;
  }

  ///
  /// \brief setCamera
  /// \param cam camera parameters (used for re-projection if point cloud is not organized)
  ///
  void setCamera(const Camera::ConstPtr cam) {
    cam_ = cam;
  }

  ///
  /// \brief setRemoveBackground
  /// \param remove_background if true, will set pixel not specified by the indices to the background color defined by
  /// its member variable background_color_
  ///
  void setRemoveBackground(bool remove_background = true) {
    remove_background_ = remove_background;
  }

  ///
  /// \brief setBackgroundColor
  /// \param background color (only used if indices are not empty and remove_background is set to true)
  ///
  void setBackgroundColor(uchar r, uchar g, uchar b) {
    background_color_ = cv::Vec3b(r, g, b);
  }

  ///
  /// \brief setMinMaxDepth (used for extracting normalized depth values)
  /// \param min_depth in meter
  /// \param max_depth in meter
  ///
  void setMinMaxDepth(float min_depth_m, float max_depth_m) {
    min_depth_m_ = min_depth_m;
    max_depth_m_ = max_depth_m;
  }

  cv::Mat extractDepth();  /// extracts depth image from pointcloud whereby depth values correspond to distance in meter
  cv::Mat getNormalizedDepth();  /// extracts depth image from pointcloud whereby depth values in meter are normalized
                                 /// uniformly from 0 (<=min_depth_m_) to 255 (>=max_depth_m_)
  cv::Mat getRGBImage();         /// returns the RGB image of the point cloud
  cv::Mat getOccupiedPixel();    /// computes a binary iamge from a point cloud which elements indicate if the
  /// corresponding pixel (sorted in row-major order) is hit by a finite point in the point
  /// cloud.

  /**
   * @brief getROI
   * @return region of interest (bounding box defined by the given indices). If no indices are given, ROI will be whole
   * image.
   */
  cv::Rect getROI() const {
    return roi_;
  }

  /**
   * @brief getIndexMap
   * @return returns the map of indices, i.e. which pixel represents which point of the unorganized(!) point cloud.
   */
  Eigen::MatrixXi getIndexMap() const {
    return index_map_;
  }

  template <typename MatType = uchar>
  cv::Mat fillMatrix(std::vector<MatType> (PCLOpenCVConverter<PointT>::*pf)(int, int));
};

/**
  * @brief computes the depth map of a point cloud with fixed size output
  * @param indices of the points belonging to the object (assumes row major indices)
  * @param width of the image/point cloud
  * @param height of the image/point cloud
  */
V4R_EXPORTS
cv::Rect computeBoundingBox(const std::vector<int> &indices, size_t width, size_t height);
}

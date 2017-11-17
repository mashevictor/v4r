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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#ifndef _OBJECT_SEGMENTATION_H
#define _OBJECT_SEGMENTATION_H

#ifndef Q_MOC_RUN
#include <QMutex>
#include <QObject>
#include <QThread>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/surface_texturing/OdmTexturing.h>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <v4r/camera_tracking_and_mapping/TSFFrame.hh>
#include <v4r/camera_tracking_and_mapping/TSFOptimizeBundle.hh>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include "CreateTrackingModel.h"
#include "params.h"
#include "sensor.h"
#endif

class ObjectSegmentation : public QThread {
  Q_OBJECT

 public:
  enum Command { FINISH_OBJECT_MODELLING, OPTIMIZE_MULTIVIEW, MAX_COMMAND, UNDEF = MAX_COMMAND };
  class Parameter {
   public:
    int morph_size;
    Parameter() : morph_size(3) {}
  };

  ObjectSegmentation();
  ~ObjectSegmentation();

  void start();
  void stop();
  bool isRunning();

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setData(const std::vector<v4r::TSFFrame::Ptr> &_map_frames, const Eigen::Matrix4f &_base_transform,
               const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max);
  void setDirectories(const std::string &_folder, const std::string &_modelname);
  const std::vector<cv::Mat_<unsigned char>> &getMasks() {
    return masks;
  }
  bool savePointClouds(const std::string &_folder, const std::string &_modelname);
  void finishModelling();
  const std::vector<std::vector<int>> &getObjectIndices() {
    return indices;
  }
  void createPointCloud(bool enable) {
    create_cloud = enable;
  }
  void createViews(bool enable) {
    create_views = enable;
  }
  void createMesh(bool enable) {
    create_mesh = enable;
  }
  void createTexturedMesh(bool enable) {
    create_tex_mesh = enable;
  }
  void createTrackingModel(bool enable) {
    create_tracking_model = enable;
  }
  void setParameter(const double &_vx_size, int _poisson_depth, int _poisson_samples) {
    voxel_size = _vx_size;
    poisson_depth = _poisson_depth;
    poisson_samples = _poisson_samples;
  }
  void useMultiviewICP(bool enable) {
    use_mvicp = enable;
  }
  void useNoiseModel(bool enable) {
    use_noise = enable;
  }
  void filterLargestCluster(bool enable) {
    filter_largest_cluster = enable;
  }

 public slots:
  void set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs);

 signals:
  void new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &image);
  void update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);
  void printStatus(const std::string &_txt);
  void update_visualization();
  void finishedModelling();

 private:
  Command cmd;
  bool m_run;

  Parameter param;

  bool create_cloud, create_views, create_mesh, create_tex_mesh, create_tracking_model;
  double voxel_size;
  int poisson_depth;
  int poisson_samples;
  double max_dist;
  int max_iterations;
  int diff_type;
  bool use_mvicp;
  bool use_noise;
  bool filter_largest_cluster;
  double edge_radius_px;
  double max_point_dist;

  cv::Mat_<double> intrinsic;
  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> dist_coeffs_opti;
  cv::Mat_<double> intrinsic_opti;

  std::vector<v4r::TSFFrame::Ptr> map_frames;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
  std::vector<cv::Mat_<unsigned char>> masks;
  std::vector<std::vector<int>> indices;
  std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> normals;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> inv_poses;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ncloud_filt;
  pcl::PointCloud<pcl::Normal>::Ptr big_normals;
  pcl::PolygonMesh mesh;
  pcl::TextureMesh tex_mesh;

  cv::Mat_<cv::Vec3b> image;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud0, tmp_cloud1, tmp_cloud2;
  cv::Mat_<unsigned char> tmp_mask;

  Eigen::Vector3f bb_min, bb_max;
  Eigen::Matrix4f object_base_transform;

  double seg_offs;

  std::string folder, model_name;

  RGBDCameraParameter cam_params;
  v4r::TSFOptimizeBundle ba;
  v4r::TSFOptimizeBundle::Parameter ba_param;
  CreateTrackingModel tm;

  void run();

  void convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat_<cv::Vec3b> &image);
  void createMaskFromROI(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, cv::Mat_<unsigned char> &mask,
                         const Eigen::Matrix4f &object_base_transform, const Eigen::Vector3f &bb_min,
                         const Eigen::Vector3f &bb_max, const double &roi_offs);
  void getMaskFromBasePlane(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose,
                            cv::Mat_<unsigned char> &mask, std::vector<int> &indices);
  void createCloudModel();
  void getSegmentedViews();
  void optimizePosesMultiviewICP();
  void createObjectCloudFilteredNguyen();

  inline bool isnan(const Eigen::Vector3f &pt);
};

/**
 * @brief ObjectSegmentation::isnan
 * @param pt
 * @return
 */
inline bool ObjectSegmentation::isnan(const Eigen::Vector3f &pt) {
  if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]))
    return true;
  return false;
}

#endif

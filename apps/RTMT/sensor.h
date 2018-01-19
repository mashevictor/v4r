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
 * @file sensor.h
 * @author Johann Prankl (prankl@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2015
 * @brief
 *
 */

#ifndef _GRAB_PCD_SENSOR_H_
#define _GRAB_PCD_SENSOR_H_

#ifndef Q_MOC_RUN
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <v4r/common/convertCloud.h>
#include <v4r/keypoints/ClusterNormalsToPlanes.h>
#include <v4r/reconstruction/KeypointSlamRGBD2.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/keypoints/impl/triple.hpp>
#include "OcclusionClustering.hh"
#include "OctreeVoxelCentroidContainerXYZRGB.hpp"
#include "params.h"
#endif

class Sensor : public QThread {
  Q_OBJECT

 public:
  class CameraLocation {
   public:
    int idx;
    int type;
    Eigen::Vector3f pt;
    Eigen::Vector3f vr;
    CameraLocation() {}
    CameraLocation(int _idx, int _type, const Eigen::Vector3f &_pt, const Eigen::Vector3f &_vr)
    : idx(_idx), type(_type), pt(_pt), vr(_vr) {}
  };
  /**
   * @brief The Surfel class
   */
  class Surfel {
   public:
    Eigen::Vector3f pt;
    Eigen::Vector3f n;
    float weight;
    float radius;
    int r, g, b;
    Surfel() : weight(0), radius(0) {}
    Surfel(const pcl::PointXYZRGB &_pt) : pt(_pt.getArray3fMap()), weight(1), radius(0), r(_pt.r), g(_pt.g), b(_pt.b) {
      if (!std::isnan(pt[0]) && !std::isnan(pt[1]) && !std::isnan(pt[2])) {
        n = -pt.normalized();
      } else {
        n = Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN());
        weight = 0;
      }
    }
  };

  Sensor();
  ~Sensor();

  typedef pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGB, pcl::octree::OctreeVoxelCentroidContainerXYZRGB<pcl::PointXYZRGB>>::AlignedPointTVector
      AlignedPointXYZRGBVector;

  void start(int cam_id = 0);
  void stop();
  void startTracker(int cam_id);
  void stopTracker();
  bool isRunning();
  void reset();
  void storeKeyframes(const std::string &_folder);
  void storeCurrentFrame(const std::string &_folder);
  void storePointCloudModel(const std::string &_folder);
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &getCameras();
  void showDepthMask(bool _draw_depth_mask);
  void selectROI(int _seed_x, int _seed_y);
  void activateROI(bool enable);

  v4r::Object::Ptr &getModel() {
    return camtracker->getModelPtr();
  }
  boost::shared_ptr<std::vector<CameraLocation>> &getTrajectory() {
    return cam_trajectory;
  }
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> &getClouds() {
    return log_clouds;
  }
  boost::shared_ptr<AlignedPointXYZRGBVector> &getAlignedCloud() {
    return oc_cloud;
  }

 public slots:
  void cam_params_changed(const RGBDCameraParameter &_cam_params);
  void cam_tracker_params_changed(const CamaraTrackerParameter &_cam_tracker_params);
  void bundle_adjustment_parameter_changed(const BundleAdjustmentParameter &param);
  void select_roi(int x, int y);
  void set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs);

 signals:
  void new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &image);
  void new_pose(const Eigen::Matrix4f &_pose);
  void update_model_cloud(const boost::shared_ptr<Sensor::AlignedPointXYZRGBVector> &_oc_cloud);
  void update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> &_cam_trajectory);
  void update_visualization();
  void printStatus(const std::string &_txt);
  void finishedOptimizeCameras(int num_cameras);
  void update_boundingbox(const std::vector<Eigen::Vector3f> &edges, const Eigen::Matrix4f &pose);
  void set_roi(const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max, const Eigen::Matrix4f &_roi_pose);

 private:
  void run();

  void CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud);
  int selectFrames(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, int cam_id, const Eigen::Matrix4f &pose,
                   std::vector<CameraLocation> &traj);
  void renewPrevCloud(const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &poses,
                      const std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> &clouds);
  void drawConfidenceBar(cv::Mat &im, const double &conf);
  void drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &im);
  void detectROI(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud);
  void getInplaneTransform(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, Eigen::Matrix4f &pose);
  void getBoundingBox(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices,
                      const Eigen::Matrix4f &pose, std::vector<Eigen::Vector3f> &bbox, Eigen::Vector3f &bb_min,
                      Eigen::Vector3f &bb_max);
  void maskCloud(const Eigen::Matrix4f &pose, const Eigen::Vector3f &bb_min, const Eigen::Vector3f &bb_max,
                 v4r::DataMatrix2D<Eigen::Vector3f> &cloud);
  void filterCloud(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const Eigen::Matrix4f &_pose,
                   pcl::PointCloud<pcl::PointXYZRGB> &_filt_cloud, const cv::Mat_<unsigned char> &_mask);
  void initCloud(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const Eigen::Matrix4f &_pose,
                 v4r::DataMatrix2D<Surfel> &_sf_cloud, Eigen::Matrix4f &_sf_pose);
  void integrateData(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, const Eigen::Matrix4f &_pose,
                     v4r::DataMatrix2D<Surfel> &_sf_cloud, Eigen::Matrix4f &_filt_pose);

  inline bool isNaN(const Eigen::Vector3f &pt);
  inline double sqr(const double &val);

  // status
  bool m_run;
  bool m_run_tracker;
  int m_cam_id;
  bool m_draw_mask;
  bool m_select_roi;
  bool m_activate_roi;

  int roi_seed_x, roi_seed_y;
  v4r::ClusterNormalsToPlanes::Plane plane;

  // parameter
  unsigned u_idle_time;
  unsigned max_queue_size;

  RGBDCameraParameter cam_params;
  CamaraTrackerParameter cam_tracker_params;
  BundleAdjustmentParameter ba_params;

  // data logging
  double cos_min_delta_angle, sqr_min_cam_distance;
  boost::shared_ptr<std::vector<CameraLocation>> cam_trajectory;
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> log_clouds;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> cameras;

  // preview
  double prev_voxel_size, prev_filter_z;
  std::vector<int> indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud, tmp_cloud2;
  boost::shared_ptr<AlignedPointXYZRGBVector> oc_cloud;

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGB, pcl::octree::OctreeVoxelCentroidContainerXYZRGB<pcl::PointXYZRGB>>::Ptr octree;

  // camera tracker
  QMutex shm_mutex;
  std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shm_clouds;

  QMutex cloud_mutex;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  v4r::DataMatrix2D<Eigen::Vector3f> kp_cloud;
  cv::Mat_<cv::Vec3b> image;

  Eigen::Matrix4f pose;
  double conf;
  int cam_id;

  v4r::KeypointSlamRGBD2::Parameter ct_param;
  v4r::KeypointSlamRGBD2::Ptr camtracker;

  boost::shared_ptr<pcl::Grabber> interface;

  // bounding box filter
  double bbox_scale_xy, bbox_scale_height, seg_offs;
  Eigen::Vector3f bb_min, bb_max;
  std::vector<Eigen::Vector3f> edges;
  Eigen::Matrix4f bbox_base_transform;

  v4r::ClusterNormalsToPlanes::Ptr pest;
  v4r::ZAdaptiveNormals::Ptr nest;

  // filtering
  v4r::OcclusionClustering occ;
  v4r::DataMatrix2D<Surfel> sf_cloud;
  Eigen::Matrix4f sf_pose;
  cv::Mat_<unsigned char> occ_mask;

  cv::Mat_<float> depth_norm;
  cv::Mat_<float> depth_weight;
  cv::Mat_<float> tmp_z;
  cv::Mat_<float> nan_z;

  std::vector<float> exp_error_lookup;
  double max_integration_frames;
  cv::Mat_<double> cam;
};

/**
 * @brief Sensor::isNaN
 * @param pt
 * @return
 */
inline bool Sensor::isNaN(const Eigen::Vector3f &pt) {
  return std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]);
}

inline double Sensor::sqr(const double &val) {
  return val * val;
}

#endif  // _GRAB_PCD_SENSOR_H_

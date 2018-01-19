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
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/camera_tracking_and_mapping/TSFGlobalCloudFilteringSimple.h>
#include <v4r/camera_tracking_and_mapping/TSFVisualSLAM.h>
#include <v4r/common/ZAdaptiveNormals.h>
#include <v4r/common/convertCloud.h>
#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>
#include <v4r/keypoints/ClusterNormalsToPlanes.h>
#include <QMutex>
#include <QThread>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/triple.hpp>
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
  void showDepthMask(bool _draw_depth_mask);
  void selectROI(int _seed_x, int _seed_y);
  void activateROI(int enable);
  void showCameras(int enable) {
    m_show_cameras = enable;
  }
  void setTSFParameter(int nb_tracked_frames_ba, int batch_size_clouds_tsf, const double &cam_dist_map,
                       const double &delta_angle_map);

  void setVignettingCalibrationFiles(const std::string &vgn_file, const std::string &crf_file) {
    tsf.setVignettingCalibrationFiles(vgn_file, crf_file);
  }

  // void createModel();
  inline const std::vector<v4r::TSFFrame::Ptr> &getMap() const {
    return tsf.getMap();
  }
  void getObjectTransform(Eigen::Matrix4f &_base_transform, Eigen::Vector3f &_bb_min, Eigen::Vector3f &_bb_max) {
    _base_transform = bbox_base_transform;
    _bb_min = bb_min + Eigen::Vector3f(0, 0, -OFFSET_BOUNDING_BOX_TRACKING + seg_offs);
    _bb_max = bb_max;
  }
  void getCameraParameter(cv::Mat &_intrinsic, cv::Mat &_dist_coeffs) {
    tsf.getCameraParameter(_intrinsic, _dist_coeffs);
  }

 public slots:
  void cam_params_changed(const RGBDCameraParameter &_cam_params);
  void select_roi(int x, int y);
  void set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs);

 signals:
  void new_image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud, const cv::Mat_<cv::Vec3b> &image);
  void new_pose(const Eigen::Matrix4f &_pose);
  void update_model_cloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud);
  void update_cam_trajectory(const boost::shared_ptr<std::vector<Sensor::CameraLocation>> &_cam_trajectory);
  void update_visualization();
  void printStatus(const std::string &_txt);
  void update_boundingbox(const std::vector<Eigen::Vector3f> &edges, const Eigen::Matrix4f &pose);

 private:
  void run();

  void CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud);
  void drawConfidenceBar(cv::Mat &im, const double &conf);
  void drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &im);
  void detectROI(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud);
  void getInplaneTransform(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, Eigen::Matrix4f &pose);
  void getBoundingBox(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices,
                      const Eigen::Matrix4f &pose, std::vector<Eigen::Vector3f> &bbox, Eigen::Vector3f &bb_min,
                      Eigen::Vector3f &bb_max);
  void maskCloud(const Eigen::Matrix4f &pose, const Eigen::Vector3f &bb_min, const Eigen::Vector3f &bb_max,
                 pcl::PointCloud<pcl::PointXYZRGB> &_cloud);

  inline bool isNaN(const Eigen::Vector3f &pt);
  inline double sqr(const double &val);

  void convertImage(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, cv::Mat &_image);

  // status
  bool m_run;
  bool m_run_tracker;
  int m_cam_id;
  bool m_draw_mask;
  bool m_select_roi;
  bool m_activate_roi;
  bool m_show_cameras;
  double upscaling;

  int roi_seed_x, roi_seed_y;
  v4r::ClusterNormalsToPlanes::Plane plane;

  // parameter
  unsigned u_idle_time;
  unsigned max_queue_size;

  RGBDCameraParameter cam_params;
  v4r::TSFVisualSLAM::Parameter param;

  // data logging
  double cos_min_delta_angle, sqr_min_cam_distance;
  boost::shared_ptr<std::vector<CameraLocation>> cam_trajectory;
  boost::shared_ptr<std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>> log_clouds;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> cameras;

  // preview
  double prev_voxel_size, prev_filter_z;
  std::vector<int> indices;

  pcl::PassThrough<pcl::PointXYZRGB> pass;

  // camera tracker
  v4r::TSFVisualSLAM tsf;
  v4r::FeatureDetector::Ptr detector;

  QMutex shm_mutex;
  std::queue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shm_clouds;

  QMutex cloud_mutex;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  v4r::DataMatrix2D<Eigen::Vector3f> kp_cloud;
  cv::Mat_<cv::Vec3b> image;

  Eigen::Matrix4f pose, inv_pose;
  double conf;
  int cam_id;

  boost::shared_ptr<pcl::Grabber> interface;

  // bounding box filter
  static const double OFFSET_BOUNDING_BOX_TRACKING;
  double bbox_scale_xy, bbox_scale_height, seg_offs;
  Eigen::Vector3f bb_min, bb_max;
  std::vector<Eigen::Vector3f> edges;
  Eigen::Matrix4f bbox_base_transform;

  v4r::ClusterNormalsToPlanes::Ptr pest;
  v4r::ZAdaptiveNormals::Ptr nest;

  cv::Mat_<double> cam;
};

/**
 * @brief Sensor::isNaN
 * @param pt
 * @return
 */
inline bool Sensor::isNaN(const Eigen::Vector3f &pt) {
  if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2]))
    return true;
  return false;
}

inline double Sensor::sqr(const double &val) {
  return val * val;
}

#endif  // _GRAB_PCD_SENSOR_H_

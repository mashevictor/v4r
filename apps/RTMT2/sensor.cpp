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

#ifndef Q_MOC_RUN
#include "sensor.h"
#include <pcl/common/io.h>

#include <v4r/common/convertImage.h>
#include <v4r/keypoints/RigidTransformationRANSAC.h>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/toString.hpp>
//#include "v4r/KeypointTools/ScopeTime.hpp"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#endif

using namespace std;

const double Sensor::OFFSET_BOUNDING_BOX_TRACKING = -0.02;

/**
 * @brief Sensor::Sensor
 */
Sensor::Sensor()
: m_run(false), m_run_tracker(false), m_cam_id(0), m_draw_mask(false), m_select_roi(false), m_activate_roi(false),
  m_show_cameras(false), upscaling(1.), roi_seed_x(320), roi_seed_y(280), u_idle_time(35 * 1000),  // 35 ms
  max_queue_size(3),  // allow a queue size of 3 clouds
  pose(Eigen::Matrix4f::Identity()), conf(0), cam_id(-1), bbox_scale_xy(1.), bbox_scale_height(1.), seg_offs(0.01),
  bb_min(Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX)), bb_max(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX)),
  bbox_base_transform(Eigen::Matrix4f::Identity()) {
  // set cameras
  cam = cv::Mat_<double>::eye(3, 3);

  cam(0, 0) = cam_params.f[0];
  cam(1, 1) = cam_params.f[1];
  cam(0, 2) = cam_params.c[0];
  cam(1, 2) = cam_params.c[1];

  cam_trajectory.reset(new std::vector<CameraLocation>());
  log_clouds.reset(new std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>());

  cos_min_delta_angle = cos(20 * M_PI / 180.);
  sqr_min_cam_distance = 1. * 1.;

  v4r::ClusterNormalsToPlanes::Parameter p_param;
  p_param.thrAngle = 45;
  p_param.inlDist = 0.01;
  p_param.minPoints = 5000;
  p_param.least_squares_refinement = true;
  p_param.smooth_clustering = false;
  p_param.thrAngleSmooth = 30;
  p_param.inlDistSmooth = 0.01;
  p_param.minPointsSmooth = 3;
  pest.reset(new v4r::ClusterNormalsToPlanes(p_param));

  v4r::ZAdaptiveNormals::Parameter n_param;
  n_param.adaptive = true;
  nest.reset(new v4r::ZAdaptiveNormals(n_param));

  // set up tracking
  tsf.setCameraParameter(cam, upscaling);
  param.tsf_filtering = true;
  param.tsf_mapping = true;
  param.pt_param.max_count = 250;
  param.pt_param.conf_tracked_points_norm = 150;
  param.map_param.refine_plk = false;
  param.map_param.detect_loops = true;
  param.map_param.nb_tracked_frames = 10;
  param.filt_param.batch_size_clouds = 20;
  param.diff_cam_distance_map = 0.5;
  param.diff_delta_angle_map = 7;
  param.filt_param.type = 3;  // 0...ori. col., 1..col mean, 2..bilin., 3..bilin col and cut off depth
  tsf.setParameter(param);

  v4r::FeatureDetector::Ptr detector(new v4r::FeatureDetector_KD_FAST_IMGD());
  tsf.setDetectors(detector, detector);
}

/**
 * @brief Sensor::~Sensor
 */
Sensor::~Sensor() {
  stop();
}

/******************************** public *******************************/

/**
 * @brief Sensor::setTSFParameter
 * @param nb_tracked_frames_ba
 * @param batch_size_clouds_tsf
 * @param cam_dist_map
 * @param delta_angle_map
 */
void Sensor::setTSFParameter(int nb_tracked_frames_ba, int batch_size_clouds_tsf, const double &cam_dist_map,
                             const double &delta_angle_map) {
  param.map_param.nb_tracked_frames = nb_tracked_frames_ba;
  param.filt_param.batch_size_clouds = batch_size_clouds_tsf;
  param.diff_cam_distance_map = cam_dist_map;
  param.diff_delta_angle_map = delta_angle_map;
  tsf.setParameter(param);
}

/**
 * @brief Sensor::getClouds
 * @return
 */
// const boost::shared_ptr< std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > >& Sensor::getClouds()
//{
//  stopTracker();
//  return log_clouds;
//}

/**
 * @brief Sensor::start
 * @param cam_id
 */
void Sensor::start(int _cam_id) {
  m_cam_id = _cam_id;
  QThread::start();
}

/**
 * @brief Sensor::stop
 */
void Sensor::stop() {
  if (m_run) {
    m_run = false;
    stopTracker();
    this->wait();
  }
  tsf.stop();
}

/**
 * @brief Sensor::startTracker
 * @param cam_id
 */
void Sensor::startTracker(int _cam_id) {
  if (!m_run)
    start(_cam_id);

  m_run_tracker = true;
}

/**
 * @brief Sensor::stopTracker
 */
void Sensor::stopTracker() {
  if (m_run_tracker) {
    m_run_tracker = false;
  }
  tsf.stop();
}

/**
 * @brief Sensor::isRunning
 * @return
 */
bool Sensor::isRunning() {
  return m_run;
}

/**
 * @brief Sensor::set_roi_params
 * @param _bbox_scale_xy
 * @param _bbox_scale_height
 * @param _seg_offs
 */
void Sensor::set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs) {
  bbox_scale_xy = _bbox_scale_xy;
  bbox_scale_height = _bbox_scale_height;
  seg_offs = _seg_offs;
}

/**
 * @brief Sensor::cam_params_changed
 * @param _cam_params
 */
void Sensor::cam_params_changed(const RGBDCameraParameter &_cam_params) {
  bool need_start = m_run;
  bool need_start_tracker = m_run_tracker;

  stop();

  cam_params = _cam_params;

  cam = cv::Mat_<double>::eye(3, 3);

  cam(0, 0) = cam_params.f[0];
  cam(1, 1) = cam_params.f[1];
  cam(0, 2) = cam_params.c[0];
  cam(1, 2) = cam_params.c[1];

  cam_trajectory.reset(new std::vector<CameraLocation>());
  log_clouds.reset(new std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>());

  cos_min_delta_angle = cos(20 * M_PI / 180.);
  sqr_min_cam_distance = 1. * 1.;
  pose = Eigen::Matrix4f::Identity();
  conf = 0.;

  tsf.setCameraParameter(cam, upscaling);

  if (need_start)
    start(cam_id);
  if (need_start_tracker)
    startTracker(cam_id);
}

/**
 * @brief Sensor::select_roi
 * @param x
 * @param y
 */
void Sensor::select_roi(int x, int y) {
  roi_seed_x = x;
  roi_seed_y = y;
  m_select_roi = true;
}

/**
 * @brief Sensor::reset
 */
void Sensor::reset() {
  bool is_run_tracker = m_run_tracker;

  stopTracker();

  tsf.reset();

  cam_trajectory.reset(new std::vector<CameraLocation>());
  log_clouds.reset(new std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>>());
  cameras.clear();

  pose.setIdentity();
  conf = 0;

  if (is_run_tracker)
    startTracker(m_cam_id);
}

/**
 * @brief Sensor::showDepthMask
 * @param _draw_depth_mask
 */
void Sensor::showDepthMask(bool _draw_depth_mask) {
  m_draw_mask = _draw_depth_mask;
}

/**
 * @brief Sensor::selectROI
 * @param _seed_x
 * @param _seed_y
 */
void Sensor::selectROI(int _seed_x, int _seed_y) {
  m_select_roi = true;
  roi_seed_x = _seed_x;
  roi_seed_y = _seed_y;
}

/**
 * @brief Sensor::activateROI
 * @param enable
 */
void Sensor::activateROI(int enable) {
  m_activate_roi = enable;
}

/*************************************** private **************************************************/

void Sensor::CallbackCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &_cloud) {
  shm_mutex.lock();

  if (shm_clouds.size() < max_queue_size) {
    shm_clouds.push(pcl::PointCloud<pcl::PointXYZRGB>::Ptr());
    shm_clouds.back().reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*_cloud, *shm_clouds.back());
  }

  shm_mutex.unlock();

  usleep(u_idle_time);
}

/**
 * @brief Sensor::run
 * main loop
 */
void Sensor::run() {
  try {
    interface.reset(new pcl::OpenNIGrabber());
  } catch (pcl::IOException e) {
    m_run = false;
    m_run_tracker = false;
    emit printStatus(std::string("Status: No OpenNI device connected!"));
    return;
  }
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f =
      boost::bind(&Sensor::CallbackCloud, this, _1);
  interface->registerCallback(f);
  interface->start();

  m_run = true;

  bool have_pose = false;
  int z = 0;
  double conf_ransac_iter = 0, conf_tracked_points = 0;
  cv::Mat_<cv::Vec3b> im_draw;
  //uint64_t ts = INT_MAX, ts_last;
  Eigen::Matrix4f filt_pose;
  pcl::PointCloud<pcl::PointXYZRGBNormal> filt_cloud;

  while (m_run) {
    // -------------------- do tracking --------------------------

    shm_mutex.lock();

    if (!shm_clouds.empty()) {
      cloud = shm_clouds.front();
      shm_clouds.pop();
    } else
      cloud.reset();

    shm_mutex.unlock();

    if (cloud.get() != 0) {
      // v4r::ScopeTime t("tracking");
      v4r::convertCloud(*cloud, kp_cloud, image);
      image.copyTo(im_draw);

      // select a roi
      if (m_select_roi)
        detectROI(kp_cloud);
      if (m_activate_roi)
        maskCloud(pose * bbox_base_transform, bb_min, bb_max, *cloud);

      // track camera
      if (m_run_tracker) {
        //        if (m_draw_mask) tsf.setDebugImage(im_draw); // debug viz
        //        else tsf.setDebugImage(cv::Mat());

        have_pose = tsf.track(*cloud, z, pose, conf_ransac_iter, conf_tracked_points);

        // debug save
        //        tsf.getFilteredCloudNormals(filt_cloud, filt_pose, ts);
        //        if (ts != ts_last && filt_cloud.points.size()>0)
        //        {
        //            pcl::io::savePCDFileBinary(std::string("log/")+v4r::toString(z)+std::string("-filt.pcd"),
        //            filt_cloud);
        //            convertImage(filt_cloud, image);
        //            cv::imwrite(std::string("log/")+v4r::toString(z)+std::string("-filt.jpg"),image);
        //            ts_last = ts;
        //        }
        if (have_pose) {
          v4r::invPose(pose, inv_pose);
          cam_trajectory->push_back(CameraLocation(z, 1, inv_pose.block<3, 1>(0, 3), inv_pose.block<3, 1>(0, 2)));
          if (m_show_cameras)
            emit update_cam_trajectory(cam_trajectory);
        }
        if (m_activate_roi)
          emit update_boundingbox(edges, pose * bbox_base_transform);
      }

      //      if (m_draw_mask) drawDepthMask(*cloud,im_draw); // debug viz
      //      drawConfidenceBar(im_draw,conf_tracked_points);
      //      emit new_image(cloud, im_draw);

      if (m_draw_mask)
        drawDepthMask(*cloud, image);
      drawConfidenceBar(image, conf_tracked_points);
      emit new_image(cloud, image);

      emit update_visualization();

      z++;
    } else
      usleep(u_idle_time / 2);
  }

  usleep(50000);
  interface->stop();
  usleep(50000);
}

/**
 * drawConfidenceBar
 */
void Sensor::drawConfidenceBar(cv::Mat &im, const double &_conf) {
  int bar_start = 50, bar_end = 200;
  int diff = bar_end - bar_start;
  int draw_end = diff * _conf;
  double col_scale = 255. / (double)diff;
  cv::Point2f pt1(0, 30);
  cv::Point2f pt2(0, 30);
  cv::Vec3b col(0, 0, 0);

  if (draw_end <= 0)
    draw_end = 1;

  for (int i = 0; i < draw_end; i++) {
    col = cv::Vec3b(255 - (i * col_scale), i * col_scale, 0);
    pt1.x = bar_start + i;
    pt2.x = bar_start + i + 1;
    cv::line(im, pt1, pt2, CV_RGB(col[0], col[1], col[2]), 8);
  }
}

/**
 * @brief Sensor::drawDepthMask
 * @param cloud
 * @param im
 */
void Sensor::drawDepthMask(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &im) {
  if ((int)_cloud.width != im.cols || (int)_cloud.height != im.rows)
    return;

  for (unsigned i = 0; i < _cloud.width * _cloud.height; i++)
    if (isnan(_cloud.points[i].x))
      im.at<cv::Vec3b>(i) = cv::Vec3b(255, 0, 0);

  //  for (unsigned i=0; i<plane.indices.size(); i++)
  //  {
  //    im.at<cv::Vec3b>(plane.indices[i]) = cv::Vec3b(255,0,0);
  //  }
}

/**
 * @brief Sensor::getInplaneTransform
 * @param pt
 * @param normal
 * @param pose
 */
void Sensor::getInplaneTransform(const Eigen::Vector3f &pt, const Eigen::Vector3f &normal, Eigen::Matrix4f &_pose) {
  _pose.setIdentity();

  Eigen::Vector3f px, py;
  Eigen::Vector3f pz = normal;

  if (pt.dot(pz) > 0)
    pz *= -1;
  px = (Eigen::Vector3f(1, 0, 0).cross(pz)).normalized();
  py = (pz.cross(px)).normalized();

  _pose.block<3, 1>(0, 0) = px;
  _pose.block<3, 1>(0, 1) = py;
  _pose.block<3, 1>(0, 2) = pz;
  _pose.block<3, 1>(0, 3) = pt;
}

/**
 * @brief Sensor::maskCloud
 * @param cloud
 * @param pose
 * @param bb_min
 * @param bb_max
 */
void Sensor::maskCloud(const Eigen::Matrix4f &_pose, const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max,
                       pcl::PointCloud<pcl::PointXYZRGB> &_cloud) {
  Eigen::Vector3f pt_glob;
  Eigen::Matrix4f inv_pose;
  v4r::invPose(_pose, inv_pose);
  Eigen::Matrix3f R = inv_pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0, 3);

  for (unsigned i = 0; i < _cloud.points.size(); i++) {
    pcl::PointXYZRGB &pt = _cloud.points[i];

    if (!isNaN(pt.getVector3fMap())) {
      pt_glob = R * pt.getVector3fMap() + t;

      if (pt_glob[0] < _bb_min[0] || pt_glob[0] > _bb_max[0] || pt_glob[1] < _bb_min[1] || pt_glob[1] > _bb_max[1] ||
          pt_glob[2] < _bb_min[2] || pt_glob[2] > _bb_max[2]) {
        pt.getVector3fMap() =
            Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN());
      }
    }
  }
}

/**
 * @brief Sensor::getBoundingBox
 * @param cloud
 * @param pose
 * @param bbox
 * @param xmin
 * @param xmax
 * @param ymin
 * @param ymax
 */
void Sensor::getBoundingBox(const v4r::DataMatrix2D<Eigen::Vector3f> &_cloud, const std::vector<int> &_indices,
                            const Eigen::Matrix4f &_pose, std::vector<Eigen::Vector3f> &bbox, Eigen::Vector3f &_bb_min,
                            Eigen::Vector3f &_bb_max) {
  Eigen::Vector3f pt, bbox_center_xy;
  double xmin, xmax, ymin, ymax, bbox_height, h_bbox_length, h_bbox_width;

  xmin = ymin = DBL_MAX;
  xmax = ymax = -DBL_MAX;

  Eigen::Matrix4f inv_pose;
  v4r::invPose(_pose, inv_pose);
  Eigen::Matrix3f R = inv_pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = inv_pose.block<3, 1>(0, 3);

  for (unsigned i = 0; i < _indices.size(); i++) {
    pt = R * _cloud[_indices[i]] + t;
    if (pt[0] > xmax)
      xmax = pt[0];
    if (pt[0] < xmin)
      xmin = pt[0];
    if (pt[1] > ymax)
      ymax = pt[1];
    if (pt[1] < ymin)
      ymin = pt[1];
  }

  h_bbox_length = bbox_scale_xy * (xmax - xmin) / 2.;
  h_bbox_width = bbox_scale_xy * (ymax - ymin) / 2.;
  bbox_height = bbox_scale_height * (xmax - xmin + ymax - ymin) / 2.;
  bbox_center_xy = Eigen::Vector3f((xmin + xmax) / 2., (ymin + ymax) / 2., 0.);

  bbox.clear();
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, 0.));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0.));

  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, -h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, h_bbox_width, bbox_height));
  bbox.push_back(bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, bbox_height));

  for (unsigned i = 0; i < 8; i += 2) {
    bbox.push_back(bbox[i]);
    bbox.push_back(bbox[i + 8]);
  }

  _bb_min = bbox_center_xy + Eigen::Vector3f(-h_bbox_length, -h_bbox_width, 0);
  _bb_max = bbox_center_xy + Eigen::Vector3f(h_bbox_length, h_bbox_width, bbox_height);
}

/**
 * @brief Sensor::detectROI
 * @param cloud
 */
void Sensor::detectROI(const v4r::DataMatrix2D<Eigen::Vector3f> &_cloud) {
  v4r::DataMatrix2D<Eigen::Vector3f> normals;

  nest->compute(_cloud, normals);
  pest->compute(_cloud, normals, roi_seed_x, roi_seed_y, plane);

  if (plane.indices.size() > 3) {
    getInplaneTransform(plane.pt, plane.normal, bbox_base_transform);
    getBoundingBox(_cloud, plane.indices, bbox_base_transform, edges, bb_min, bb_max);
    emit update_boundingbox(edges, bbox_base_transform);
    bb_min += Eigen::Vector3f(0, 0, OFFSET_BOUNDING_BOX_TRACKING);
  }

  // cout<<"[Sensor::detectROI] roi plane nb pts: "<<plane.indices.size()<<endl;

  m_activate_roi = true;
  m_select_roi = false;
}

void Sensor::convertImage(const pcl::PointCloud<pcl::PointXYZRGBNormal> &_cloud, cv::Mat &_image) {
  _image = cv::Mat_<cv::Vec3b>(_cloud.height, _cloud.width);

  for (unsigned v = 0; v < _cloud.height; v++) {
    for (unsigned u = 0; u < _cloud.width; u++) {
      cv::Vec3b &cv_pt = _image.at<cv::Vec3b>(v, u);
      const pcl::PointXYZRGBNormal &pt = _cloud(u, v);

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}

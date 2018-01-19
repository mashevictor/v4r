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

/**
 * TODO:
 * - if the loop is closed return the updated pose to the tracker and data integration
 * - test tracking confidence
 */

#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <v4r/camera_tracking_and_mapping/TSFVisualSLAM.h>
#include <v4r/common/convertImage.h>
#include <v4r/keypoints/impl/invPose.hpp>

#include "opencv2/highgui/highgui.hpp"

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
TSFVisualSLAM::TSFVisualSLAM(const Parameter &p)
: remove_vignetting(false), tsf_width(0), tsf_height(0), upscaling_ratio(1.), last_ts_filt(-1) {
  setParameter(p);
  tsfPoseTracker.setData(&data);
  tsfMapping.setData(&data);
  last_pose_map(0, 0) = std::numeric_limits<float>::quiet_NaN();
}

TSFVisualSLAM::~TSFVisualSLAM() {}

bool TSFVisualSLAM::selectFrame(const Eigen::Matrix4f &pose0, const Eigen::Matrix4f &pose1) {
  invPose(pose0, inv_pose0);
  invPose(pose1, inv_pose1);
  if ((inv_pose0.block<3, 1>(0, 2).dot(inv_pose1.block<3, 1>(0, 2)) >= cos_diff_delta_angle_map) &&
      (inv_pose0.block<3, 1>(0, 3) - inv_pose1.block<3, 1>(0, 3)).squaredNorm() <= sqr_diff_cam_distance_map)
    return false;
  return true;
}

/**
 * @brief TSFVisualSLAM::setScaledCameraParamter
 * @param width
 * @param height
 */
void TSFVisualSLAM::setScaledCameraParamter(int width, int height) {
  tsf_width = upscaling_ratio * width;
  tsf_height = upscaling_ratio * height;
  tsf_intrinsic = cv::Mat_<double>::eye(3, 3);
  tsf_intrinsic(0, 0) = upscaling_ratio * intrinsic(0, 0);
  tsf_intrinsic(1, 1) = upscaling_ratio * intrinsic(1, 1);
  tsf_intrinsic(0, 2) = upscaling_ratio * intrinsic(0, 2);
  tsf_intrinsic(1, 2) = upscaling_ratio * intrinsic(1, 2);
  tsFilter.setCameraParameterTSF(tsf_intrinsic, tsf_width, tsf_height);
  tsfMapping.setCameraParameter(tsf_intrinsic);
}

/**
 * @brief TSFilterCloudsXYZRGB::scaleBrightness
 * @param im_lin
 * @param scale
 */
void TSFVisualSLAM::scaleBrightness(cv::Mat &_im_lin, const float &scale) {
  for (unsigned i = 0; i < (unsigned)_im_lin.rows * _im_lin.cols; i++) {
    _im_lin.at<cv::Point3f>(i) *= scale;
  }
}

/**
 * @brief TSFilterCloudsXYZRGB::removeVignetting
 * @param frames
 */
void TSFVisualSLAM::removeVignetting(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &im) {
  if (vr.get() != 0 && rr.get() != 0) {
    // pcl::ScopeTime t("TSFilterCloudsXYZRGB::removeVignetting");
    convertImage(_cloud, im);
    rr->inverseMap(im, im_lin);
    vr->remove(im_lin, im_lin_corr);
    scaleBrightness(im_lin_corr, param.im_brightness_scale);
    rr->directMap(im_lin_corr, im);
  }
}

/***************************************************************************************/

/**
 * @brief TSFVisualSLAM::stop
 */
void TSFVisualSLAM::stop() {
  tsfPoseTracker.stop();
  tsfMapping.stop();
  tsFilter.stop();
}

/**
 * reset
 */
void TSFVisualSLAM::reset() {
  //  data.lock();   // i have to think about it....
  stop();
  tsfPoseTracker.reset();
  tsfMapping.reset();
  tsFilter.reset();
  data.reset();
  last_ts_filt = -1;
  last_pose_map(0, 0) = std::numeric_limits<float>::quiet_NaN();
  //  data.unlock();
}

/**
 * @brief TSFVisualSLAM::track
 * @param cloud
 * @param pose
 */
bool TSFVisualSLAM::track(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const uint64_t &timestamp,
                          Eigen::Matrix4f &pose, double &conf_ransac_iter, double &conf_tracked_points) {
  if (intrinsic.empty())
    throw std::runtime_error("[TSFVisualSLAM::track] Camera parameter not available!");

  if (!tsfPoseTracker.isStarted())
    tsfPoseTracker.start();

  if (!tsFilter.isStarted() && (param.tsf_filtering || param.tsf_mapping)) {
    tsFilter.setCameraParameter(intrinsic);
    setScaledCameraParamter(cloud.width, cloud.height);
    tsFilter.start();
  }

  if (!tsfMapping.isStarted() && param.tsf_mapping) {
    setScaledCameraParamter(cloud.width, cloud.height);
    tsfMapping.start();
  }

  // should be save ;-)
  if (remove_vignetting) {
    removeVignetting(cloud, data.image);
  } else
    convertImage(cloud, data.image);

  cv::cvtColor(data.image, data.gray, CV_BGR2GRAY);

  if (!dbg.empty())
    tsfPoseTracker.dbg = dbg;

  // the tracker does not copy data, we need to lock the shared memory
  data.lock();
  data.have_pose = (data.cloud.points.size() == 0 ? true : false);
  data.cloud = cloud;
  if (remove_vignetting) {
    setImage(data.image, data.cloud);
  }
  data.timestamp = timestamp;
  tsfPoseTracker.track(conf_ransac_iter, conf_tracked_points);
  pose = data.pose;
  bool have_pose = data.have_pose;
  if (!have_pose)
    data.lost_track = true;

  // if filtering is activated add frames to the batch filter
  if (param.tsf_filtering) {
    tsFilter.addCloud(data.cloud, pose, timestamp, have_pose);
  }

  // if mapping is activated collect select keyframes for mapping
  if (param.tsf_filtering && param.tsf_mapping) {
    tsFilter.lock();
    if (fabs((tsFilter.getFiltTimestamp() - last_ts_filt)) > std::numeric_limits<double>::epsilon()) {
      if (tsFilter.getFiltTimestamp() != 0 &&
          (std::isnan(last_pose_map(0, 0)) || selectFrame(last_pose_map, tsFilter.getFiltPose()))) {
        if (tsFilter.getFiltCloud().data.size() > 0) {
          data.map_frames.push(
              TSFFrame::Ptr(new TSFFrame(-1, tsFilter.getFiltPose(), tsFilter.getFiltCloud(), !data.lost_track)));
          data.lost_track = false;
          last_pose_map = tsFilter.getFiltPose();
          last_ts_filt = tsFilter.getFiltTimestamp();
        }
      }
    }
    tsFilter.unlock();
  }

  data.unlock();

  return have_pose;
}

/**
 * @brief TSFVisualSLAM::getFilteredCloudNormals
 * @param cloud
 * @param timestamp
 */
int TSFVisualSLAM::getFilteredCloudNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, Eigen::Matrix4f &pose,
                                           uint64_t &timestamp) {
  double ts;
  int nb = tsFilter.getFilteredCloudNormals(cloud, pose, ts);
  timestamp = (uint64_t)ts;
  return nb;
}

/**
 * @brief TSFVisualSLAM::getFilteredCloud
 * @param cloud
 * @param timestamp
 */
int TSFVisualSLAM::getFilteredCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Matrix4f &pose,
                                    uint64_t &timestamp) {
  double ts;
  int nb = tsFilter.getFilteredCloud(cloud, pose, ts);
  timestamp = (uint64_t)ts;
  return nb;
}

/**
 * @brief TSFVisualSLAM::getSurfelCloud
 * @param cloud
 * @param pose
 * @param timestamp
 */
int TSFVisualSLAM::getSurfelCloud(v4r::DataMatrix2D<Surfel> &cloud, Eigen::Matrix4f &pose, uint64_t &timestamp) {
  double ts;
  int nb = tsFilter.getSurfelCloud(cloud, pose, ts);
  timestamp = (uint64_t)ts;
  return nb;
}

/**
 * @brief TSFVisualSLAM::setParameter
 * @param p
 */
void TSFVisualSLAM::setParameter(const Parameter &p) {
  param = p;
  tsfPoseTracker.setParameter(p.pt_param);
  tsfMapping.setParameter(p.map_param);
  tsFilter.setParameter(p.filt_param);
  sqr_diff_cam_distance_map = param.diff_cam_distance_map * param.diff_cam_distance_map;
  cos_diff_delta_angle_map = cos(param.diff_delta_angle_map * M_PI / 180.);
}

/**
 * setCameraParameter
 * for the camera pose tracker
 */
void TSFVisualSLAM::setCameraParameter(const cv::Mat &_intrinsic, double _upscaling_ratio) {
  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else
    intrinsic = _intrinsic;

  tsfPoseTracker.setCameraParameter(_intrinsic);
  upscaling_ratio = _upscaling_ratio;
  tsf_width = tsf_height = 0;

  reset();
}

/**
 * @brief TSFVisualSLAM::setCRFfile
 * @param file
 */
void TSFVisualSLAM::setVignettingCalibrationFiles(const std::string &vgn_file, const std::string &crf_file) {
  if (vgn_file.size() > 0 && crf_file.size() > 0) {
    remove_vignetting = true;
    vr.reset(new radical::VignettingResponse(vgn_file));
    rr.reset(new radical::RadiometricResponse(crf_file));
  } else {
    remove_vignetting = false;
    vr = boost::shared_ptr<radical::VignettingResponse>();
    rr = boost::shared_ptr<radical::RadiometricResponse>();
  }
}

/**
 * @brief TSFVisualSLAM::setDetectors
 * @param _detector
 * @param _descEstimator
 */
void TSFVisualSLAM::setDetectors(const FeatureDetector::Ptr &_detector, const FeatureDetector::Ptr &_descEstimator) {
  tsfMapping.setDetectors(_detector, _descEstimator);
}

/**
 * @brief TSFVisualSLAM::optimizeMap
 */
void TSFVisualSLAM::optimizeMap() {
  tsfMapping.optimizeMap();
}

void TSFVisualSLAM::getCameraParameter(cv::Mat &_intrinsic, cv::Mat &_dist_coeffs) {
  if (param.tsf_mapping) {
    tsfMapping.getCameraParameter(_intrinsic, _dist_coeffs);
  } else {
    tsf_intrinsic.copyTo(_intrinsic);
    _dist_coeffs = cv::Mat();
  }
}
}

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

#ifndef KP_TSF_VISUAL_SLAM_HH
#define KP_TSF_VISUAL_SLAM_HH

#include <float.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <radical/radiometric_response.h>
#include <radical/vignetting_response.h>
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/core/macros.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <v4r/camera_tracking_and_mapping/TSFMapping.hh>
#include <v4r/camera_tracking_and_mapping/TSFPoseTrackerKLT.hh>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include "v4r/camera_tracking_and_mapping/TSFilterCloudsXYZRGB.h"

namespace v4r {

/**
 * TSFVisualSLAM
 */
class V4R_EXPORTS TSFVisualSLAM {
 public:
  /**
   * Parameter
   */
  class Parameter {
   public:
    bool tsf_filtering;  // activates the batch filtering (temporal smoothing), batch size is specified in filt_param
    bool tsf_mapping;    // collects keyframes and optionally optimizes the poses (post processing if optimizeMap is
                         // called)
    double diff_cam_distance_map;  // minimum distance the camera must move to select a keyframe
    double diff_delta_angle_map;   // or minimum angle the camera need to rotate to be selected
    float im_brightness_scale;

    TSFPoseTrackerKLT::Parameter pt_param;
    TSFilterCloudsXYZRGB::Parameter filt_param;
    TSFMapping::Parameter map_param;
    Parameter()
    : tsf_filtering(true), tsf_mapping(true), diff_cam_distance_map(0.5), diff_delta_angle_map(7.),
      im_brightness_scale(0.8) {
      //      // HACK for testing
      //      path_to_vgn_file = std::string("ps1080.1403190110-sa-cam.vgn");
      //      path_to_crf_file = std::string("ps1080.1403190110-sa-cam.crf");
    }
  };

 private:
  Parameter param;
  bool remove_vignetting;
  int tsf_width, tsf_height;
  double upscaling_ratio;

  double sqr_diff_cam_distance_map;
  double cos_diff_delta_angle_map;

  cv::Mat_<double> intrinsic, tsf_intrinsic;

  TSFData data;

  double last_ts_filt;
  Eigen::Matrix4f last_pose_map;

  TSFPoseTrackerKLT tsfPoseTracker;
  TSFMapping tsfMapping;
  TSFilterCloudsXYZRGB tsFilter;

  cv::Mat dbg;

  Eigen::Matrix4f inv_pose0, inv_pose1;

  cv::Mat im_lin, im_lin_corr;

  boost::shared_ptr<radical::VignettingResponse> vr;
  boost::shared_ptr<radical::RadiometricResponse> rr;

  bool selectFrame(const Eigen::Matrix4f &pose0, const Eigen::Matrix4f &pose1);
  void setScaledCameraParamter(int width, int height);
  void removeVignetting(const pcl::PointCloud<pcl::PointXYZRGB> &_cloud, cv::Mat &im);
  void scaleBrightness(cv::Mat &_im_lin, const float &scale);

 public:
  TSFVisualSLAM(const Parameter &p = Parameter());
  ~TSFVisualSLAM();

  /**
   * @brief reset data structures
   */
  void reset();

  /**
   * @brief stop background threads
   */
  void stop();

  /**
   * @brief track the camera pose of RGB-D frames
   * @param cloud
   * @param timestamp need to be an incremented number
   * @param pose
   * @param conf_ransac_iter
   * @param conf_tracked_points tracked point ratio
   * @return
   */
  bool track(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const uint64_t &timestamp, Eigen::Matrix4f &pose,
             double &conf_ransac_iter, double &conf_tracked_points);

  /**
   * @brief setDebugImage draws the tracked keypoint motion
   * @param _dbg
   */
  void setDebugImage(const cv::Mat &_dbg) {
    dbg = _dbg;
  }

  /**
   * @brief get methodes provide filtered and batch optimized RGB-D frames
   * timestamp referes to the corresponding tracking timestamp. It also indicates if a new smoothed frame is available
   */
  int getFilteredCloudNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, Eigen::Matrix4f &pose,
                              uint64_t &timestamp);
  int getFilteredCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Matrix4f &pose, uint64_t &timestamp);
  int getSurfelCloud(v4r::DataMatrix2D<Surfel> &cloud, Eigen::Matrix4f &pose, uint64_t &timestamp);

  /**
   * @brief setCameraParameter
   * @param _intrinsic intrinsic camera parameter provided for tracking the organized point cloud
   * @param _upscaling_ratio used to create upscaled RGBD frames in case Parameter::tsf_filtering or
   * Parameter::tsf_mapping is activated
   *                         e.g. 1.25 => 640->800, 1.6 => 640->1024
   */
  void setCameraParameter(const cv::Mat &_intrinsic, double _upscaling_ratio = 1.);

  /**
   * @brief setParameter includes activation of TSF filtering and mapping
   * @param p
   */
  void setParameter(const Parameter &p);

  void setVignettingCalibrationFiles(const std::string &vgn_file, const std::string &crf_file);

  /**
   * @brief setDetectors needed for mapping/ loop closing
   * @param _detector
   * @param _descEstimator
   */
  void setDetectors(const FeatureDetector::Ptr &_detector, const FeatureDetector::Ptr &_descEstimator);

  /**
   * @brief optimizeMap
   * does a bundle adjustment
   */
  void optimizeMap();

  /**
   * @brief transformMap transforms the coordinate system of the map (can only be used for postprocessing)
   * @param transform
   */
  void transformMap(const Eigen::Matrix4f &transform) {
    tsfMapping.transformMap(transform);
  }

  /**
   * @brief getCameraParameter
   * returns upscaled and optimized camera parameter (optimized in case optimizing intrinsics is activated and
   * 'optimizeMap' is called before)
   * @param _intrinsic
   * @param _dist_coeffs
   */
  void getCameraParameter(cv::Mat &_intrinsic, cv::Mat &_dist_coeffs);

  /**
   * @brief getMap
   * @return keyframes and poses in case Parameter::tsf_mapping is activated (optimized if optimizeMap is called before)
   */
  inline const std::vector<TSFFrame::Ptr> &getMap() const {
    return tsfMapping.getMap();
  }

  /** returns keypoints (debug method) **/
  inline const std::vector<std::vector<Eigen::Vector3d>> &getOptiPoints() const {
    return tsfMapping.getOptiPoints();
  }

  typedef boost::shared_ptr<::v4r::TSFVisualSLAM> Ptr;
  typedef boost::shared_ptr<::v4r::TSFVisualSLAM const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

}  //--END--

#endif

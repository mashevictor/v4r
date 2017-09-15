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

#ifndef KP_KEYPOINT_SLAM_RGBD2_HH
#define KP_KEYPOINT_SLAM_RGBD2_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <v4r/common/impl/SmartPtr.hpp>

#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/reconstruction/KeyframeManagementRGBD2.h>
#include <v4r/reconstruction/ProjLKPoseTrackerRT.h>
#include <v4r/reconstruction/LKPoseTrackerRT.h>
#include <v4r/reconstruction/KeypointPoseDetectorRT.h>
#include <v4r/core/macros.h>
#include <v4r/common/impl/DataMatrix2D.hpp>
#include <v4r/features/FeatureDetector_KD_FAST_IMGD.h>

namespace v4r
{

/**
 * KeypointSlamRGBD2
 */
class V4R_EXPORTS KeypointSlamRGBD2
{
public:

  /**
   * Parameter
   */
  class Parameter
  {
  public:
    double conf_reinit;  // 0.05
    bool do_inc_pyr_lk;  // true
    bool close_loops;    // true
    double min_conf;     // 0.3
    int min_conf_cnt;    // 2
    double add_keyframe_conf; // 0.6
    double add_keyframe_angle; // 10 (slam 5)
                // deviation of the mean view ray (e.g. 10°, needed for reinit with keypoints)
    double add_keyframe_view_overlap;// 0.8, (slam 0.9) overlap of the keyframes (e.g. 50%) 
    KeyframeManagementRGBD2::Parameter om_param;
    KeypointPoseDetectorRT::Parameter kd_param;
    LKPoseTrackerRT::Parameter lk_param;
    ProjLKPoseTrackerRT::Parameter kt_param;
    FeatureDetector_KD_FAST_IMGD::Parameter det_param;
    Parameter( double _conf_reinit=0.05, bool _do_inc_pyr_lk=true, bool _close_loops=true,
      double _min_conf=0.3, int _min_conf_cnt=2, double _add_keyframe_conf=0.4,
      double _add_keyframe_angle=10, double _add_keyframe_view_overlap=0.8,
      const KeyframeManagementRGBD2::Parameter &_om_param = KeyframeManagementRGBD2::Parameter(),
      const KeypointPoseDetectorRT::Parameter &_kd_param = KeypointPoseDetectorRT::Parameter(),
      const LKPoseTrackerRT::Parameter &_lk_param = LKPoseTrackerRT::Parameter(),
      const ProjLKPoseTrackerRT::Parameter &_kt_param = ProjLKPoseTrackerRT::Parameter(),
      const FeatureDetector_KD_FAST_IMGD::Parameter &_det_param = FeatureDetector_KD_FAST_IMGD::Parameter(250, 1.44, 2, 17, 2) ) 
    : conf_reinit(_conf_reinit), do_inc_pyr_lk(_do_inc_pyr_lk), close_loops(_close_loops),
      min_conf(_min_conf), min_conf_cnt(_min_conf_cnt), add_keyframe_conf(_add_keyframe_conf),
      add_keyframe_angle(_add_keyframe_angle), add_keyframe_view_overlap(_add_keyframe_view_overlap),
      om_param(_om_param), kd_param(_kd_param), lk_param(_lk_param), kt_param(_kt_param), det_param(_det_param) { }
  };

  

private:
  Parameter param;
  double rad_add_keyframe_angle;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;
  
  cv::Mat_<unsigned char> im_gray;

  ObjectView::Ptr view;
  Eigen::Matrix4f view_pose, delta_pose;

  std::vector< std::pair<int,cv::Point2f> > im_pts;

  double conf;
  int conf_cnt;
  Eigen::Matrix4f pose;

  int new_kf_1st_frame, new_kf_2nd_frame;
  
  KeyframeManagementRGBD2::Ptr om;

  KeypointPoseDetectorRT::Ptr kpDetector;
  ProjLKPoseTrackerRT::Ptr kpTracker;
  LKPoseTrackerRT::Ptr lkTracker;

  bool addKeyframe(const ObjectView &view, const Eigen::Matrix4f &delta_pose, const Eigen::Matrix4f &view_pose, 
        const Eigen::Matrix4f &pose, int width, int height, double conf);
  double viewPointChange(const Eigen::Vector3f &pt, const Eigen::Matrix4f &inv_pose1, 
        const Eigen::Matrix4f &inv_pose2);


public:
  cv::Mat dbg;

  KeypointSlamRGBD2(const KeypointSlamRGBD2::Parameter &p=KeypointSlamRGBD2::Parameter());
  ~KeypointSlamRGBD2();

  bool track(const cv::Mat &image, const DataMatrix2D<Eigen::Vector3f> &cloud, Eigen::Matrix4f &current_pose, double &current_conf, int &cam_id);

  void setKeyframe(int idx, Eigen::Matrix4f &pose);
  int getKeyframe() { return (view.get()==0?-1:view->idx); }

  inline Object::Ptr &getModelPtr() { return om->getModelPtr(); }
  inline Object &getModel() { return om->getModel(); }
  
  inline void lock(){ om->lock(); }        // threaded object management, so we need to lock 
  inline void unlock() { om->unlock(); }
  void stopObjectManagement() { om->stop(); }

  void reset();

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  void setMinDistAddProjections(const double &dist) { om->setMinDistAddProjections(dist); }

  typedef SmartPtr< ::v4r::KeypointSlamRGBD2> Ptr;
  typedef SmartPtr< ::v4r::KeypointSlamRGBD2 const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif


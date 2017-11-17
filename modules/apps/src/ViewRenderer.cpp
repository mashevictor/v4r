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

#include <v4r_modules.h>
#include <v4r/apps/ViewRenderer.h>

#include <stdio.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#ifdef HAVE_V4R_RENDERING
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <v4r/rendering/depthmapRenderer.h>
#endif

#include <v4r/common/miscellaneous.h>
#include <v4r/io/filesystem.h>
#include <pcl/point_types.h>

#include <glog/logging.h>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r {

namespace apps {

void ViewRenderer::render(const bf::path &input_path, const bf::path &out_path) {
#ifdef HAVE_V4R_RENDERING
  v4r::DepthmapRenderer renderer(cam_->getWidth(), cam_->getHeight());
  renderer.setIntrinsics(cam_->getFocalLengthX(), cam_->getFocalLengthY(), cam_->getCx(), cam_->getCy());

  v4r::DepthmapRendererModel model(input_path.string(), "", param_.autoscale);

  if (model.hasColor() || model.hasTexture())
    LOG(INFO) << "Model file has color.";
  else
    LOG(INFO) << "Model file has no color.";

  renderer.setModel(&model);

  std::vector<Eigen::Vector3f> sphere = renderer.createSphere(param_.radius, param_.subdivisions);

  if (param_.upperHemisphere) {
    std::vector<Eigen::Vector3f> upper;
    for (size_t i = 0; i < sphere.size(); i++) {
      if (sphere[i][2] > 0) {
        upper.push_back(sphere[i]);
      }
    }
    sphere = upper;
  }
  LOG(INFO) << "Rendering file " << input_path.string();

  if (!sphere.empty())
    v4r::io::createDirIfNotExist(out_path);

  for (size_t i = 0; i < sphere.size(); i++) {
    // get point from list
    const Eigen::Vector3f &point = sphere[i];
    // get a camera pose looking at the center:
    const Eigen::Matrix4f &orientation = renderer.getPoseLookingToCenterFrom(point);

    renderer.setCamPose(orientation);

    float visible;
    cv::Mat color;

    // create and save the according pcd files
    std::stringstream ss;
    ss << "cloud_" << i << ".pcd";

    Eigen::Matrix4f cam_pose;

    bf::path output_fn = out_path / ss.str();
    if (model.hasColor() || model.hasTexture()) {
      const pcl::PointCloud<pcl::PointXYZRGB> cloud = renderer.renderPointcloudColor(visible);

      pcl::io::savePCDFileBinary(output_fn.string(), cloud);

      //TODO avoid duplication of this block
      cam_pose = v4r::RotTrans2Mat4f(cloud.sensor_orientation_, cloud.sensor_origin_);

      std::string indices_fn = output_fn.string();
      boost::replace_last(indices_fn, "cloud", "object_indices");
      boost::replace_last(indices_fn, ".pcd", ".txt");
      std::ofstream indices_f(indices_fn);
      for (size_t i = 0; i < cloud.points.size(); i++)
        if (pcl::isFinite(cloud.points[i]))
          indices_f << i << std::endl;
      indices_f.close();

    } else {
      const pcl::PointCloud<pcl::PointXYZ> cloud = renderer.renderPointcloud(visible);
      pcl::io::savePCDFileBinary(output_fn.string(), cloud);

      cam_pose = v4r::RotTrans2Mat4f(cloud.sensor_orientation_, cloud.sensor_origin_);

      std::string indices_fn = output_fn.string();
      boost::replace_last(indices_fn, "cloud", "object_indices");
      boost::replace_last(indices_fn, ".pcd", ".txt");
      std::ofstream indices_f(indices_fn);
      for (size_t i = 0; i < cloud.points.size(); i++)
        if (pcl::isFinite(cloud.points[i]))
          indices_f << i << std::endl;
      indices_f.close();
    }

    std::string pose_fn = output_fn.string();
    boost::replace_last(pose_fn, "cloud", "pose");
    boost::replace_last(pose_fn, ".pcd", ".txt");
    std::ofstream pose_f(pose_fn);
    for (size_t u = 0; u < 4; u++)
      for (size_t v = 0; v < 4; v++)
        pose_f << cam_pose(u, v) << " ";
    pose_f.close();

    LOG(INFO) << "Saved data points to " << output_fn.string() << ".";
  }

#else
  LOG(ERROR) << "V4R rendering is not available. Did you enable the v4r_rendering module during compilation? Skipping view rendering!";
#endif
}

}
}
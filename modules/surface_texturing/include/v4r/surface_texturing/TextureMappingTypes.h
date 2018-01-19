/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef V4R_TEXTURE_MAPPING_TYPES_H_
#define V4R_TEXTURE_MAPPING_TYPES_H_

#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>

namespace v4r {
namespace texture_mapping {

/** \brief Structure to store camera pose and focal length.
  *
  * One can assign a value to focal_length, to be used along
  * both camera axes or, optionally, axis-specific values
  * (focal_length_w and focal_length_h). Optionally, one can
  * also specify center-of-focus using parameters
  * center_w and center_h. If the center-of-focus is not
  * specified, it will be set to the geometric center of
  * the camera, as defined by the width and height parameters.
  */
struct Camera {
  Camera()
  : pose(), focal_length(), focal_length_w(-1), focal_length_h(-1), center_w(-1), center_h(-1), height(), width(),
    texture_file() {}
  Eigen::Affine3f pose;
  double focal_length;
  double focal_length_w;  // optional
  double focal_length_h;  // optinoal
  double center_w;        // optional
  double center_h;        // optional
  double height;
  double width;
  std::string texture_file;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** \brief Structure that links a uv coordinate to its 3D point and face.
  */
struct UvIndex {
  UvIndex() : idx_cloud(), idx_face() {}
  int idx_cloud;  // Index of the PointXYZ in the camera's cloud
  int idx_face;   // Face corresponding to that projection
};

typedef std::vector<Camera, Eigen::aligned_allocator<Camera>> CameraVector;
}
}  // namespace v4r

#endif /* V4R_TEXTUREMESH_H_ */

/*
 * viewport_checker.cpp
 *
 *  Created on: 24.11.2015
 *      Author: ivelas
 */

#include <glog/logging.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <v4r/change_detection/viewport_checker.h>

namespace v4r {

template <class PointType>
int ViewVolume<PointType>::computeVisible(const typename pcl::PointCloud<PointType>::Ptr input,
                                          std::vector<bool> &mask) const {
  pcl::PointCloud<PointType> input_transformed;
  pcl::transformPointCloud(*input, input_transformed, sensor_pose.inverse());

  int visible_count = 0;
  CHECK(input->size() == mask.size());
  for (size_t i = 0; i < input_transformed.size(); i++) {
    bool isIn = in(input_transformed[i]);
    if (isIn) {
      visible_count++;
    }
    if (!mask[i]) {
      mask[i] = isIn;
    }
  }
  return visible_count;
}

template <class PointT>
pcl::PointCloud<pcl::PointXYZ> ViewVolume<PointT>::getBorders() const {
  double x_near = max_sin_h_angle * min_dist;
  double y_near = max_sin_v_angle * min_dist;
  double x_far = max_sin_h_angle * max_dist;
  double y_far = max_sin_v_angle * max_dist;

  pcl::PointCloud<pcl::PointXYZ> borders;
  borders.push_back(pcl::PointXYZ(x_near, y_near, min_dist));
  borders.push_back(pcl::PointXYZ(-x_near, y_near, min_dist));
  borders.push_back(pcl::PointXYZ(-x_near, -y_near, min_dist));
  borders.push_back(pcl::PointXYZ(x_near, -y_near, min_dist));

  borders.push_back(pcl::PointXYZ(x_far, y_far, max_dist));
  borders.push_back(pcl::PointXYZ(-x_far, y_far, max_dist));
  borders.push_back(pcl::PointXYZ(-x_far, -y_far, max_dist));
  borders.push_back(pcl::PointXYZ(x_far, -y_far, max_dist));

  pcl::transformPointCloud(borders, borders, sensor_pose);
  return borders;
}

template <class PointType>
void ViewportChecker<PointType>::getVisibles(const typename pcl::PointCloud<PointType>::Ptr input,
                                             typename pcl::PointCloud<PointType>::Ptr visible,
                                             typename pcl::PointCloud<PointType>::Ptr nonVisible) const {
  std::vector<bool> mask(input->size(), false);
  for (auto &vol : volumes) {
    vol.computeVisible(input, mask);
  }
  for (size_t i = 0; i < mask.size(); i++) {
    if (mask[i])
      visible->push_back(input->at(i));
    else
      nonVisible->push_back(input->at(i));
  }
}

template class V4R_EXPORTS ViewVolume<pcl::PointXYZRGB>;
template class V4R_EXPORTS ViewportChecker<pcl::PointXYZRGB>;
}

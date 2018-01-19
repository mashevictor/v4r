#include <v4r/segmentation/plane_utils.h>
#include <v4r/segmentation/segmenter_organized_connected_component.h>

#include <pcl/common/angles.h>
#include <pcl/pcl_config.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

template <typename PointT>
void OrganizedConnectedComponentSegmenter<PointT>::segment() {
  clusters_.clear();

  pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  labels->points.resize(scene_->points.size());
  for (pcl::Label &p : labels->points)
    p.label = 1;

#if PCL_VERSION_COMPARE(<=, 1, 8, 1)
  auto euclidean_cluster_comp = boost::make_shared<pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>>();
  euclidean_cluster_comp->setAngularThreshold(pcl::deg2rad(param_.angular_threshold_deg_));
  std::vector<bool> exclude_labels(scene_->points.size(), false);
  euclidean_cluster_comp->setExcludeLabels(exclude_labels);
#else
  auto euclidean_cluster_comp = boost::make_shared<pcl::EuclideanClusterComparator<PointT, pcl::Label>>();
#endif
  euclidean_cluster_comp->setInputCloud(scene_);
  euclidean_cluster_comp->setLabels(labels);
  euclidean_cluster_comp->setDistanceThreshold(param_.distance_threshold_, true);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> seg(euclidean_cluster_comp);
  seg.setInputCloud(scene_);
  seg.segment(euclidean_labels, euclidean_label_indices);

  for (size_t i = 0; i < euclidean_label_indices.size(); i++) {
    if (euclidean_label_indices[i].indices.size() >= param_.min_cluster_size_)
      clusters_.push_back(euclidean_label_indices[i].indices);
  }
}

#define PCL_INSTANTIATE_OrganizedConnectedComponentSegmenter(T) \
  template class V4R_EXPORTS OrganizedConnectedComponentSegmenter<T>;
PCL_INSTANTIATE(OrganizedConnectedComponentSegmenter, PCL_XYZ_POINT_TYPES)
}

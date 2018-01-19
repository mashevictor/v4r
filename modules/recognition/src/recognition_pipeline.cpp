#include <glog/logging.h>
#include <v4r/recognition/recognition_pipeline.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

template <typename PointT>
std::vector<std::pair<std::string, float>> RecognitionPipeline<PointT>::elapsed_time_;

template <typename PointT>
RecognitionPipeline<PointT>::StopWatch::~StopWatch() {
  boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
  float elapsed_time = static_cast<float>(((end_time - start_time_).total_milliseconds()));
  VLOG(1) << desc_ << " took " << elapsed_time << " ms.";
  elapsed_time_.push_back(std::pair<std::string, float>(desc_, elapsed_time));
}

template <typename PointT>
void RecognitionPipeline<PointT>::recognize() {
  elapsed_time_.clear();
  obj_hypotheses_.clear();
  CHECK(scene_) << "Input scene is not set!";

  if (needNormals())
    CHECK(scene_normals_ && scene_->points.size() == scene_normals_->points.size())
        << "Recognizer needs normals but they are not set!";

  do_recognize();
}

#define PCL_INSTANTIATE_RecognitionPipeline(T) template class V4R_EXPORTS RecognitionPipeline<T>;
PCL_INSTANTIATE(RecognitionPipeline, (pcl::PointXYZRGB))
}

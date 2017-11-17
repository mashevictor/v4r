#include <glog/logging.h>
#include <pcl/point_types.h>
#include <v4r/io/filesystem.h>
#include <v4r/recognition/source.h>
#include <pcl/impl/instantiate.hpp>

namespace v4r {

template <typename PointT>
void Source<PointT>::init(const bf::path &model_database_path){
  std::vector<std::string> categories;
  if (param_.has_categories_)
    categories = io::getFoldersInDirectory(model_database_path);
  else
    categories.push_back("");

  for (const std::string &cat : categories) {
    const bf::path class_path = model_database_path / cat;
    const std::vector<std::string> instance_names = io::getFoldersInDirectory(class_path);

    LOG(INFO) << "Loading " << instance_names.size() << " object models from folder " << class_path.string() << ". ";
    for (const std::string instance_name : instance_names) {
      typename Model<PointT>::Ptr obj(new Model<PointT>);
      obj->id_ = instance_name;
      obj->class_ = cat;

      const bf::path object_dir = class_path / instance_name / param_.view_folder_name_;
      const std::string view_pattern = ".*" + param_.view_prefix_ + ".*.pcd";
      std::vector<std::string> training_view_filenames =
          io::getFilesInDirectory(object_dir, view_pattern, false);

      LOG(INFO) << " ** loading model (class: " << cat << ", instance: " << instance_name << ") with "
                << training_view_filenames.size() << " views. ";

      for (size_t v_id = 0; v_id < training_view_filenames.size(); v_id++) {
        typename TrainingView<PointT>::Ptr v(new TrainingView<PointT>);
        bf::path view_filename = object_dir / training_view_filenames[v_id];
        v->filename_ = view_filename.string();

        v->pose_filename_ = v->filename_;
        boost::replace_last(v->pose_filename_, param_.view_prefix_, param_.pose_prefix_);
        boost::replace_last(v->pose_filename_, ".pcd", ".txt");

        v->indices_filename_ = v->filename_;
        boost::replace_last(v->indices_filename_, param_.view_prefix_, param_.indices_prefix_);
        boost::replace_last(v->indices_filename_, ".pcd", ".txt");

        obj->addTrainingView(v);
      }

      if (!param_.has_categories_) {
        bf::path model3D_path = class_path / instance_name / param_.name_3D_model_;
        obj->initialize(model3D_path);
      }
      addModel(obj);
    }
  }
}

template class V4R_EXPORTS Source<pcl::PointXYZ>;
template class V4R_EXPORTS Source<pcl::PointXYZRGB>;
}

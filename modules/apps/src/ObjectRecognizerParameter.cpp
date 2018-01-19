#include <glog/logging.h>
#include <v4r/apps/ObjectRecognizerParameter.h>

namespace v4r {
namespace apps {

void ObjectRecognizerParameter::load(const bf::path &filename) {
  CHECK(v4r::io::existsFile(filename)) << "Given config file " << filename.string()
                                       << " does not exist! Current working directory is "
                                       << boost::filesystem::current_path().string() + ".";

  LOG(INFO) << "Loading parameters from file " << filename.string();

  try {
    std::ifstream ifs(filename.string());
    boost::archive::xml_iarchive ia(ifs);
    ia >> boost::serialization::make_nvp("ObjectRecognizerParameter", *this);
    ifs.close();
  } catch (const std::exception &e) {
    LOG(ERROR) << e.what() << std::endl;
    exit(0);
  }

  validate();
}

void ObjectRecognizerParameter::output() const {
  std::stringstream ss;
  boost::archive::text_oarchive oa(ss);
  oa << boost::serialization::make_nvp("ObjectRecognizerParameter", *this);

  LOG(INFO) << "Loaded Parameters: " << std::endl << ss.str();
}

void ObjectRecognizerParameter::validate() {
  if (global_feature_types_.size() != classification_methods_.size()) {
    size_t minn = std::min<size_t>(global_feature_types_.size(), classification_methods_.size());

    LOG(ERROR) << "The given parameter for feature types, classification methods "
               << "and configuration files for global recognition are not the same size!";
    if (minn)
      LOG(ERROR) << " Will only use the first " << minn << " global recognizers for which all three elements are set! ";
    else
      LOG(ERROR) << "Global recognition is disabled!";

    global_feature_types_.resize(minn);
    classification_methods_.resize(minn);
  }
}
}
}
#include <v4r/io/eigen.h>
#include <iostream>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <glog/logging.h>

namespace v4r {
namespace io {

bool writeMatrixToFile(const boost::filesystem::path &path, const Eigen::Matrix4f &matrix) {
  std::ofstream out(path.string().c_str());
  if (!out) {
    std::cout << "Cannot open file.\n";
    return false;
  }

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 4; j++) {
      out << matrix(i, j);
      if (!(i == 3 && j == 3))
        out << " ";
    }
  }
  out.close();

  return true;
}

Eigen::Matrix4f readMatrixFromFile(const boost::filesystem::path &path, int padding) {
  CHECK (boost::filesystem::exists(path) || boost::filesystem::is_regular_file(path))
    << "Given file path " << path.string() << " to read matrix does not exist!" ;

  std::ifstream in(path.string().c_str());

  char linebuf[1024];
  in.getline(linebuf, 1024);
  std::string line(linebuf);
  std::vector<std::string> strs_2;
  boost::split(strs_2, line, boost::is_any_of(" "));

  Eigen::Matrix4f matrix;
  for (int i = 0; i < 16; i++)
    matrix(i / 4, i % 4) = static_cast<float>(atof(strs_2[padding + i].c_str()));

  in.close();
  return matrix;
}

}
}

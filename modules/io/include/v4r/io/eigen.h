#pragma once

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <v4r/core/macros.h>

namespace v4r {
namespace io {

V4R_EXPORTS Eigen::Matrix4f readMatrixFromFile(const boost::filesystem::path &file, int padding = 0);
V4R_EXPORTS bool writeMatrixToFile(const boost::filesystem::path &, const Eigen::Matrix4f &);

}
}


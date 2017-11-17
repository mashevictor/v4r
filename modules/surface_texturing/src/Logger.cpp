#include <v4r/surface_texturing/Logger.h>

namespace v4r {

using namespace std;

Logger::Logger(bool isPrintingInCout) : isPrintingInCout_(isPrintingInCout) {}

Logger::~Logger() {}

void Logger::printToFile(std::string filePath) {
  std::ofstream file(filePath.c_str(), std::ios::binary);
  file << logStream_.str();
  file.close();
}

bool Logger::isPrintingInCout() const {
  return isPrintingInCout_;
}

void Logger::setIsPrintingInCout(bool isPrintingInCout) {
  isPrintingInCout_ = isPrintingInCout;
}
}

v4r_build_external_project("Gtest"
  URL "https://github.com/google/googletest/archive/release-1.8.0.tar.gz"
  URL_HASH SHA1=e7e646a6204638fe8e87e165292b8dd9cd4c36ed
  CMAKE_ARGS "-DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DBUILD_GTEST=ON -DBUILD_GMOCK=OFF"
)

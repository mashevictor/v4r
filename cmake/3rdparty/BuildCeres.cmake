# Glog is a mandatory dependency and should have been found by now.
# Extract the include directory and library location from the "glog" imported target.
get_target_property(_library glog IMPORTED_LOCATION)
get_target_property(_include_dirs glog INTERFACE_INCLUDE_DIRECTORIES)

v4r_build_external_project("Ceres"
  URL "http://ceres-solver.org/ceres-solver-1.13.0.tar.gz"
  URL_HASH SHA256=1DF490A197634D3AAB0A65687DECD362912869C85A61090FF66F073C967A7DCD
  CMAKE_ARGS "-DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DGLOG_INCLUDE_DIR=${_include_dirs} -DGLOG_LIBRARY=${_library}"
)

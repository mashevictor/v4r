v4r_build_external_project("Radical"
  URL https://github.com/taketwo/radical/archive/1.0.0.tar.gz
  URL_HASH SHA256=672751353e83472ba81bdde0bfad550784dd1e66502d88721aa9b406a5a90ea8
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> -DBUILD_APPS=OFF"
)

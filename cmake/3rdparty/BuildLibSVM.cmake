v4r_build_external_project("LibSVM"
  URL "https://github.com/cjlin1/libsvm/archive/v320.tar.gz"
  URL_HASH SHA256=e4be7fc8d2e7cb65feae4d6387967484c01c9dc818e7dfff515f663728a6f2ca
  CONFIGURE_COMMAND NO
  BUILD_COMMAND "make lib"
  BUILD_IN_SOURCE
  INSTALL_COMMAND "mkdir -p <INSTALL_DIR>/include/libsvm && "
                  "cp <SOURCE_DIR>/svm.h <INSTALL_DIR>/include/libsvm && "
                  "mkdir -p <INSTALL_DIR>/lib && "
                  "cp <SOURCE_DIR>/libsvm.so.2 <INSTALL_DIR>/lib"
)


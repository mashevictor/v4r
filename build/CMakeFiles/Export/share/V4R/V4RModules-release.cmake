#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "v4r_core" for configuration "Release"
set_property(TARGET v4r_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_core PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_core.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_core.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_core "${_IMPORT_PREFIX}/lib/libv4r_core.so.2.0.6" )

# Import target "v4r_io" for configuration "Release"
set_property(TARGET v4r_io APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_io PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;boost;eigen;glog;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_io.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_io.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_io )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_io "${_IMPORT_PREFIX}/lib/libv4r_io.so.2.0.6" )

# Import target "v4r_semantic_segmentation" for configuration "Release"
set_property(TARGET v4r_semantic_segmentation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_semantic_segmentation PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;boost;eigen;opencv;pcl;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_semantic_segmentation.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_semantic_segmentation.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_semantic_segmentation )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_semantic_segmentation "${_IMPORT_PREFIX}/lib/libv4r_semantic_segmentation.so.2.0.6" )

# Import target "v4r_common" for configuration "Release"
set_property(TARGET v4r_common APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_common PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;boost;eigen;glog;opencv;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_common.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_common.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_common )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_common "${_IMPORT_PREFIX}/lib/libv4r_common.so.2.0.6" )

# Import target "v4r_keypoints" for configuration "Release"
set_property(TARGET v4r_keypoints APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_keypoints PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;boost;eigen;glog;opencv;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_keypoints.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_keypoints.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_keypoints )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_keypoints "${_IMPORT_PREFIX}/lib/libv4r_keypoints.so.2.0.6" )

# Import target "v4r_ml" for configuration "Release"
set_property(TARGET v4r_ml APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_ml PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;boost;eigen;glog;libsvm;opencv;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_ml.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_ml.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_ml )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_ml "${_IMPORT_PREFIX}/lib/libv4r_ml.so.2.0.6" )

# Import target "v4r_rendering" for configuration "Release"
set_property(TARGET v4r_rendering APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_rendering PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;assimp;boost;eigen;glew;glm;glog;opencv;opengl;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_rendering.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_rendering.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_rendering )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_rendering "${_IMPORT_PREFIX}/lib/libv4r_rendering.so.2.0.6" )

# Import target "v4r_segmentation" for configuration "Release"
set_property(TARGET v4r_segmentation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_segmentation PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;boost;eigen;glog;opencv;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_segmentation.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_segmentation.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_segmentation )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_segmentation "${_IMPORT_PREFIX}/lib/libv4r_segmentation.so.2.0.6" )

# Import target "v4r_attention_segmentation" for configuration "Release"
set_property(TARGET v4r_attention_segmentation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_attention_segmentation PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;boost;eigen;glog;opencv;opennurbs;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_attention_segmentation.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_attention_segmentation.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_attention_segmentation )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_attention_segmentation "${_IMPORT_PREFIX}/lib/libv4r_attention_segmentation.so.2.0.6" )

# Import target "v4r_change_detection" for configuration "Release"
set_property(TARGET v4r_change_detection APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_change_detection PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;boost;eigen;glog;opencv;pcl;pcl_1_8;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_change_detection.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_change_detection.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_change_detection )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_change_detection "${_IMPORT_PREFIX}/lib/libv4r_change_detection.so.2.0.6" )

# Import target "v4r_features" for configuration "Release"
set_property(TARGET v4r_features APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_features PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;boost;eigen;glog;opencv;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_features.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_features.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_features )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_features "${_IMPORT_PREFIX}/lib/libv4r_features.so.2.0.6" )

# Import target "v4r_reconstruction" for configuration "Release"
set_property(TARGET v4r_reconstruction APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_reconstruction PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_features;boost;ceres;eigen;glog;metslib;opencv;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_reconstruction.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_reconstruction.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_reconstruction )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_reconstruction "${_IMPORT_PREFIX}/lib/libv4r_reconstruction.so.2.0.6" )

# Import target "v4r_registration" for configuration "Release"
set_property(TARGET v4r_registration APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_registration PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_features;boost;ceres;edt;eigen;glog;opencv;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_registration.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_registration.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_registration )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_registration "${_IMPORT_PREFIX}/lib/libv4r_registration.so.2.0.6" )

# Import target "v4r_tracking" for configuration "Release"
set_property(TARGET v4r_tracking APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_tracking PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_features;v4r_reconstruction;v4r_registration;boost;ceres;edt;eigen;glog;metslib;opencv;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_tracking.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_tracking.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_tracking )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_tracking "${_IMPORT_PREFIX}/lib/libv4r_tracking.so.2.0.6" )

# Import target "v4r_object_modelling" for configuration "Release"
set_property(TARGET v4r_object_modelling APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_object_modelling PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_features;v4r_registration;boost;ceres;edt;eigen;glog;opencv;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_object_modelling.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_object_modelling.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_object_modelling )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_object_modelling "${_IMPORT_PREFIX}/lib/libv4r_object_modelling.so.2.0.6" )

# Import target "v4r_recognition" for configuration "Release"
set_property(TARGET v4r_recognition APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_recognition PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_ml;v4r_rendering;v4r_segmentation;v4r_features;v4r_reconstruction;v4r_registration;assimp;boost;ceres;edt;eigen;glew;glm;glog;libsvm;metslib;opencv;opengl;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_recognition.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_recognition.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_recognition )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_recognition "${_IMPORT_PREFIX}/lib/libv4r_recognition.so.2.0.6" )

# Import target "v4r_apps" for configuration "Release"
set_property(TARGET v4r_apps APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_apps PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_ml;v4r_rendering;v4r_segmentation;v4r_change_detection;v4r_features;v4r_reconstruction;v4r_registration;v4r_recognition;assimp;boost;ceres;edt;eigen;glew;glm;glog;libsvm;metslib;opencv;opengl;pcl;pcl_1_8;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_apps.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_apps.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_apps )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_apps "${_IMPORT_PREFIX}/lib/libv4r_apps.so.2.0.6" )

# Import target "v4r_camera_tracking_and_mapping" for configuration "Release"
set_property(TARGET v4r_camera_tracking_and_mapping APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_camera_tracking_and_mapping PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_ml;v4r_rendering;v4r_segmentation;v4r_features;v4r_reconstruction;v4r_registration;v4r_recognition;assimp;boost;ceres;edt;eigen;glew;glm;glog;libsvm;metslib;opencv;opengl;pcl;pcl_1_8;radical;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_camera_tracking_and_mapping.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_camera_tracking_and_mapping.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_camera_tracking_and_mapping )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_camera_tracking_and_mapping "${_IMPORT_PREFIX}/lib/libv4r_camera_tracking_and_mapping.so.2.0.6" )

# Import target "v4r_surface_texturing" for configuration "Release"
set_property(TARGET v4r_surface_texturing APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(v4r_surface_texturing PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "v4r_core;v4r_io;v4r_common;v4r_keypoints;v4r_ml;v4r_rendering;v4r_segmentation;v4r_features;v4r_reconstruction;v4r_registration;v4r_recognition;v4r_camera_tracking_and_mapping;assimp;boost;ceres;edt;eigen;glew;glm;glog;libsvm;metslib;opencv;opengl;pcl;pcl_1_8;radical;siftgpu;X11;Xrandr;Xinerama;Xcursor;Xxf86vm;dl;m;pthread;rt"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libv4r_surface_texturing.so.2.0.6"
  IMPORTED_SONAME_RELEASE "libv4r_surface_texturing.so.2.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS v4r_surface_texturing )
list(APPEND _IMPORT_CHECK_FILES_FOR_v4r_surface_texturing "${_IMPORT_PREFIX}/lib/libv4r_surface_texturing.so.2.0.6" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

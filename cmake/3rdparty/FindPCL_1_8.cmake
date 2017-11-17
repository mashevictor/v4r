# We only support building bundled version of PCL_1_8, so we know for sure where it is installed
set(PCL_1_8_INCLUDE_DIRS ${V4R_3P_PCL_1_8_INSTALL_DIR}/include)
set(PCL_1_8_LIBRARIES pcl_1_8)
set(HAVE_PCL_1_8 TRUE)

# Filter out optimized/debug keywords, because INTERFACE_LINK_LIBRARIES can not digest them
set(_pcl_libraries "${PCL_LIBRARIES}")
v4r_list_filterout(_pcl_libraries "optimized|debug")
v4r_list_unique(_pcl_libraries)

add_library(pcl_1_8 STATIC IMPORTED GLOBAL)
set_target_properties(pcl_1_8 PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${PCL_1_8_INCLUDE_DIRS}"
  IMPORTED_LOCATION "${V4R_3P_PCL_1_8_INSTALL_DIR}/lib/libpcl_1_8.a"
  INTERFACE_LINK_LIBRARIES_RELEASE "${_pcl_link}"
)


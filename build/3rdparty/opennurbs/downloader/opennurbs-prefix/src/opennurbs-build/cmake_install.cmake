# Install script for directory: /home/victor/software/v4r/3rdparty/opennurbs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/victor/software/v4r/build/3rdparty/opennurbs/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opennurbs" TYPE FILE FILES
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_3dm_attributes.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_3dm.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_3dm_properties.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_3dm_settings.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_annotation2.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_annotation.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_arccurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_arc.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_archive.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_array_defs.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_array.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_base32.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_base64.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_beam.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_bezier.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_bitmap.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_bounding_box.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_box.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_brep.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_circle.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_color.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_compress.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_cone.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_crc.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_curve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_curveonsurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_curveproxy.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_cylinder.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_defines.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_detail.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_dimstyle.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_dll_resource.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_ellipse.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_error.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_evaluate_nurbs.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_extensions.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_font.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_fpoint.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_fsp_defs.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_fsp.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_geometry.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_group.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_hatch.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_hsort_template.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_instance.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_intersect.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_knot.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_layer.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_light.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_linecurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_line.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_linestyle.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_linetype.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_lookup.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_mapchan.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_material.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_math.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_matrix.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_memory.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_mesh.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_nurbscurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_nurbssurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_object.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_object_history.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_objref.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_offsetsurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_optimize.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_plane.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_planesurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_pluginlist.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_pointcloud.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_pointgeometry.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_pointgrid.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_point.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_polycurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_polyedgecurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_polylinecurve.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_polyline.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_qsort_template.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_rand.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_rendering.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_revsurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_rtree.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_sphere.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_string.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_sumsurface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_surface.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_surfaceproxy.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_system.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_textlog.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_texture.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_texture_mapping.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_torus.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_unicode.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_userdata.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_uuid.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_version.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_viewport.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_workspace.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_xform.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/opennurbs_zlib.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/crc32.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/deflate.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/inffast.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/inffixed.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/inflate.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/inftrees.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/trees.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/zconf.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/zlib.h"
    "/home/victor/software/v4r/3rdparty/opennurbs/zutil.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/victor/software/v4r/build/3rdparty/opennurbs/downloader/opennurbs-prefix/src/opennurbs-build/libopennurbs.a")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/victor/software/v4r/build/3rdparty/opennurbs/downloader/opennurbs-prefix/src/opennurbs-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

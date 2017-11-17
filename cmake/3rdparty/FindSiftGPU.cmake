# We only support building SiftGPU from source, so we know for sure where it is installed
set(SIFTGPU_INCLUDE_DIRS ${V4R_3P_SIFTGPU_INSTALL_DIR}/include)
set(SIFTGPU_LIBRARIES siftgpu)
set(SIFTGPU_VERSION "v400")
set(HAVE_SIFTGPU TRUE)

add_library(siftgpu STATIC IMPORTED GLOBAL)
set_target_properties(siftgpu PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${SIFTGPU_INCLUDE_DIRS}"
  IMPORTED_LOCATION "${V4R_3P_SIFTGPU_INSTALL_DIR}/lib/libsiftgpu.a"
  INTERFACE_LINK_LIBRARIES "GLEW;IL;glut;GL;X11"
)

# We assume that the interface link libraries are available system-wide.
# This is a valid assumption because by the time we reach this point
# SiftGPU has been built already (and it is linked against them).

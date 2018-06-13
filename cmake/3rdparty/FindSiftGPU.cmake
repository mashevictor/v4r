# We only support building SiftGPU from source, so we know for sure where it is installed
v4r_add_imported_library(siftgpu
  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
  IMPORTED_LOCATION "/usr/local//lib/libsiftgpu.a"
  INTERFACE_LINK_LIBRARIES glew opengl "IL;glut;X11"
)
set(SIFTGPU_VERSION "v400")
set(HAVE_SIFTGPU TRUE)

# We assume that the interface link libraries are available system-wide.
# This is a valid assumption because by the time we reach this point
# SiftGPU has been built already (and it is linked against them).

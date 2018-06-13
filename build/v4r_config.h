/* V4R compiled as static or dynamic libs */
#define BUILD_SHARED_LIBS

/* Compile for 'real' NVIDIA GPU architectures */
#define CUDA_ARCH_BIN ""

/* Create PTX or BIN for 1.0 compute capability */
/* #undef CUDA_ARCH_BIN_OR_PTX_10 */

/* NVIDIA GPU features are used */
#define CUDA_ARCH_FEATURES ""

/* Compile for 'virtual' NVIDIA PTX architectures */
#define CUDA_ARCH_PTX ""

/* C= */
/* #undef HAVE_CSTRIPES */

/* NVidia Cuda Runtime API*/
/* #undef HAVE_CUDA */

/* Eigen Matrix & Linear Algebra Library */
#define HAVE_EIGEN

/* Define to 1 if you have the <inttypes.h> header file. */
/* #undef HAVE_INTTYPES_H */

/* OpenGL support*/
#define HAVE_OPENGL

/* Qt support */
#define HAVE_QT

/* Qt OpenGL support */
/* #undef HAVE_QT_OPENGL */

/* Define if your processor stores words with the most significant byte
   first (like Motorola and SPARC, unlike Intel and VAX). */
/* #undef WORDS_BIGENDIAN */

#define HAVE_GLM
#define HAVE_SIFTGPU
#define HAVE_LIBSVM
#define HAVE_METSLIB
#define HAVE_PCL
#define HAVE_OPENCV
#define HAVE_CERES
#define HAVE_ASSIMP
/* #undef HAVE_CAFFE */



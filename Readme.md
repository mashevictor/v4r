[![build status](https://rgit.acin.tuwien.ac.at/v4r/v4r/badges/master/build.svg)](https://rgit.acin.tuwien.ac.at/v4r/v4r/commits/master)

The V4R library is an open source project from the [Vision4Robotics (V4R)](http://v4r.acin.tuwien.ac.at) group at the ACIN institute, TU Vienna. It contains various state-of-the-art Computer Vision methods and applications especially designed for 3D point cloud processing with a strong focus on robotics applications.

# Dependencies:  
V4R is stronlgy based on RGB-D data and uses [OpenCV](http://opencv.org/)  and [PCL](http://pointclouds.org/) to process data. Here is a list of **required** components:

| Name | Version | Licence |
| ------------- |:-------------:| -----:|
| [OpenCV](http://opencv.org/)  | 2.4+  or 3.x | 3-clause BSD |
| [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  | 3+ | MPL2 |
| [Boost](http://www.boost.org/)  | 1.48+ | Boost Software License |
| [PCL](http://pointclouds.org/)  | 1.7.x or 1.8 | BSD |
| [glog](https://code.google.com/archive/p/google-glog/)  |  | New BSD |
| [CMake](https://cmake.org)  | 3.5.1+ | 3-clause BSD |


This is a list of **optional** components:

| Name | Version | Licence |  required[*r*] or optionally-used[*o*] by following V4R modules | Comments |
| ------------- |:-------------:| -----:|-----:|-----:|
| [Ceres](http://ceres-solver.org/)\* | 1.13 | New BSD |  reconstruction[*r*], camera_tracking_and_mapping[*r*], registration[*r*], recognition[*o*] | |
| [LibSVM](http://www.csie.ntu.edu.tw/~cjlin/libsvm/)  | 3.20 | modified BSD  |  ml[*o*] |
| [SiftGPU](https://github.com/pitzer/SiftGPU)\*   | v400 |  Copyright (c) 2007 University of North Carolina at Chapel Hill (see [license](https://github.com/pitzer/SiftGPU/blob/master/license.txt)) |  features[*o*] | required if SIFT is enabled; can be replaced by *non-free* OpenCV SIFT implementation. This option is enabled if BUILD_SIFTGPU is disabled in cmake. |
| EDT\*  |  |  BSD |  features, recognition | computes the Euclidean Distance Transform for point clouds |
| [METSlib](https://projects.coin-or.org/metslib)\*  |  |  GPLv3 or EPL-1.0 (ex CPL) |   recognition[*r*] | required for object hypotheses verification |
| [openNURBS](opennurbs.org)\*  |  |  No restrictions. Neither copyright nor copyleft restrictions apply.  |   attention_segmentation[*r*] | to model smooth surfaces|
| [Caffe](https://github.com/BVLC/caffe)   |  |  2-Clause BSD  |   features[*o*] | Deep Learning Framework used for feature extraction.  |
| [OpenGL](https://www.opengl.org/resources/libraries/)   |  | Software developers do not need to license OpenGL to use it in their applications. They can simply link to a library provided by a hardware vendor. |  rendering[*r*] |  |
| [Assimp](http://assimp.sourceforge.net/)   |  | BSD |   rendering[*r*] | used to import 3D shape models for various common file types|
| [GLEW](http://glew.sourceforge.net/)   |  | Modified BSD, MIT |   rendering[*r*] |  |
| [GLM](https://glm.g-truc.net/0.9.8/index.html)  |  | MIT | rendering[*r*], RTMT[*r*], RTMT2[*r*]| |
| [Qt](https://www.qt.io/)  | 5 |  GPL and LGPLv3  | RTMT[*r*], RTMT2[*r*], Object Ground Truth Annotator[*r*]| provides graphical user interface (GUI) |
| [ODM](http://opendronemap.org/)\*  |  |  GPLv3 | RTMT2[*r*], surface_texturing[*r*]| creates textured surfaces for object models |

\* these libraries will be installed automatically during V4R's build process (either by downloading source code from a given URL or by using code copies inside V4R)


The library itself is independent of any robot operating system such as ROS. To use it inside ROS, you can find wrappers in a seperate [V4R ROS  wrappers repository](https://rgit.acin.tuwien.ac.at/v4r/v4r_ros_wrappers), which can be placed inside the normal catkin workspace.

# Installation:  

## From Ubuntu Package  
First enable the V4R apt repositories (trusty only for now) with the following commands.

 * Add the following line to your `/etc/apt/sources.list` file or in a new file (eg. `v4r.list`) in the `/etc/apt/sources.list.d`  directory.  
 `deb [arch=amd64] https://rwiki.acin.tuwien.ac.at/apt/v4r-release trusty main`

* Add the gpg key I used to sign the release to your keyring
`wget -qO - https://rwiki.acin.tuwien.ac.at/apt/v4r-release/Public.key | sudo apt-key add -`

* Update and install  
`sudo apt-get update && sudo apt-get install v4r`

## From Source  
```
cd ~/somewhere
git clone 'https://rgit.acin.tuwien.ac.at/v4r/v4r.git'
cd v4r
./setup.sh
mkdir build
cd build
cmake ..
make
sudo make install #optional
```

## ROS Interfaces  
In order to use V4R in ROS, use the [v4r_ros_wrappers](https://rgit.acin.tuwien.ac.at/v4r/v4r_ros_wrappers).  


## Notes
### Caffe
If you want to use CNN feature extraction, you need to install the Caffe library. We recommend to use CMake-based installation for Caffe and provide the install folder to V4R's cmake call as
```
cmake .. -DCaffe_DIR=/your_caffe_ws/build/install/share/Caffe
```


### Ceres
To avoid issues with Ceres when building shared libraries, we recommend to build and install Ceres from source.

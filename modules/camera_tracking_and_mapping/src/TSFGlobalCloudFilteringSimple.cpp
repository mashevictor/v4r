/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <v4r/camera_tracking_and_mapping/TSFGlobalCloudFilteringSimple.h>
#include <v4r/common/convertImage.h>
#include <v4r/camera_tracking_and_mapping/OctreeVoxelCentroidContainerXYZRGBNormal.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include "pcl/common/time.h"
#include "pcl/common/transforms.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"

namespace v4r {

using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
TSFGlobalCloudFilteringSimple::TSFGlobalCloudFilteringSimple(const Parameter &p)
: base_transform(Eigen::Matrix4f::Identity()), bb_min(Eigen::Vector3f(-FLT_MAX, -FLT_MAX, -FLT_MAX)),
  bb_max(Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX)) {
  setParameter(p);
}

TSFGlobalCloudFilteringSimple::~TSFGlobalCloudFilteringSimple() {}

/**
 * @brief TSFGlobalCloudFilteringSimple::getMask
 * @param sf_cloud
 * @param mask
 */
void TSFGlobalCloudFilteringSimple::getMask(const v4r::DataMatrix2D<v4r::Surfel> &sf_cloud,
                                            cv::Mat_<unsigned char> &mask) {
  tmp_mask = cv::Mat_<unsigned char>::ones(sf_cloud.rows, sf_cloud.cols) * 255;

  for (int v = 0; v < sf_cloud.rows; v++) {
    for (int u = 0; u < sf_cloud.cols; u++) {
      const v4r::Surfel &sf = sf_cloud(v, u);
      if (isnan(sf.pt[0]) || isnan(sf.pt[1]) || isnan(sf.pt[2]))
        tmp_mask(v, u) = 0;
    }
  }

  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * param.erosion_size + 1, 2 * param.erosion_size + 1),
                                cv::Point(param.erosion_size, param.erosion_size));
  cv::erode(tmp_mask, mask, element);
  //  cv::imshow("tmp_mask", tmp_mask);
  //  cv::imshow("mask", mask);
  //  cv::waitKey(0);
}

/**
 * @brief TSFGlobalCloudFilteringSimple::filterCluster
 * @param cloud
 * @param cloud
 */
void TSFGlobalCloudFilteringSimple::filterCluster(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
                                                  pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_filt) {
  // pcl::ScopeTime t("[TSFGlobalCloudFilteringSimple::filterCluster]");
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
  ec.setClusterTolerance(0.01);  // 2cm
  ec.setMinClusterSize(100);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int idx = -1;
  int max = 0;
  for (unsigned i = 0; i < cluster_indices.size(); i++) {
    if (((int)cluster_indices[i].indices.size()) > max) {
      max = cluster_indices[i].indices.size();
      idx = i;
    }
  }

  cloud_filt.clear();

  if (idx == -1)
    return;

  const pcl::PointCloud<pcl::PointXYZRGBNormal> &ref = *cloud;
  pcl::PointIndices &inds = cluster_indices[idx];
  cloud_filt.points.resize(inds.indices.size());
  for (unsigned i = 0; i < inds.indices.size(); i++)
    cloud_filt.points[i] = ref.points[inds.indices[i]];
  cloud_filt.width = cloud_filt.points.size();
  cloud_filt.height = 1;
  cloud_filt.is_dense = true;
}

/***************************************************************************************/

/**
 * @brief TSFGlobalCloudFilteringSimple::getGlobalCloudMasked
 * @param frames
 * @param masks
 * @param cloud
 */
void TSFGlobalCloudFilteringSimple::getGlobalCloudMasked(const std::vector<v4r::TSFFrame::Ptr> &frames,
                                                         const std::vector<cv::Mat_<unsigned char>> &masks,
                                                         pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud) {
  // pcl::ScopeTime mt("[TSFGlobalCloudFilteringSimple::getGlobalCloudFiltered]");
  cloud.clear();
  Eigen::Matrix4f inv_pose;
  Eigen::Vector3f pt;
  cv::Point2f im_pt;
  Eigen::Matrix3f R, inv_R;
  Eigen::Vector3f t, inv_t;
  pcl::PointXYZRGBNormal pcl_pt;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PointCloud<pcl::PointXYZRGBNormal> &ref = *tmp_cloud;
  cv::Mat_<unsigned char> mask;

  pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGBNormal, pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>::Ptr
      octree;
  typedef pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGBNormal,
      pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>::AlignedPointTVector
      AlignedPointXYZRGBNormalVector;
  boost::shared_ptr<AlignedPointXYZRGBNormalVector> oc_cloud;
  oc_cloud.reset(new AlignedPointXYZRGBNormalVector());
  octree.reset(new pcl::octree::OctreePointCloudVoxelCentroid<
               pcl::PointXYZRGBNormal, pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>(
      param.voxel_size));
  octree->setResolution(param.voxel_size);

  double cos_rad_thr_delta_angle = cos(param.thr_delta_angle * M_PI / 180.);

  for (unsigned i = 0; i < frames.size(); i++) {
    ref.clear();
    const v4r::TSFFrame &frame = *frames[i];
    R = frame.delta_cloud_rgb_pose.topLeftCorner<3, 3>();
    t = frame.delta_cloud_rgb_pose.block<3, 1>(0, 3);
    v4r::invPose(frame.pose * base_transform, inv_pose);
    inv_R = inv_pose.topLeftCorner<3, 3>();
    inv_t = inv_pose.block<3, 1>(0, 3);

    getMask(frame.sf_cloud, mask);
    const cv::Mat_<unsigned char> &m2 = masks[i];

    for (int v = 0; v < frame.sf_cloud.rows; v++) {
      for (int u = 0; u < frame.sf_cloud.cols; u++) {
        if (mask(v, u) < 128 || m2(v, u) < 128)
          continue;
        const v4r::Surfel &sf = frame.sf_cloud(v, u);
        if (isnan(sf.pt[0]) || isnan(sf.n[0]))
          continue;
        pt = R * sf.pt + t;
        if (dist_coeffs.empty())
          v4r::projectPointToImage(&pt[0], &intrinsic(0, 0), &im_pt.x);
        else
          v4r::projectPointToImage(&pt[0], &intrinsic(0, 0), &dist_coeffs(0), &im_pt.x);
        if (im_pt.x >= 0 && im_pt.y >= 0 && im_pt.x < frame.sf_cloud.cols && im_pt.y < frame.sf_cloud.rows) {
          pcl_pt.getVector3fMap() = inv_R * sf.pt + inv_t;
          if (pcl_pt.x < bb_min[0] || pcl_pt.x > bb_max[0] || pcl_pt.y < bb_min[1] || pcl_pt.y > bb_max[1] ||
              pcl_pt.z < bb_min[2] || pcl_pt.z > bb_max[2])
            continue;
          if (sf.weight >= param.thr_weight && sf.n.dot(-sf.pt.normalized()) > cos_rad_thr_delta_angle) {
            pcl_pt.getNormalVector3fMap() = inv_R * sf.n;
            getInterpolatedRGB(frame.sf_cloud, im_pt, pcl_pt.r, pcl_pt.g, pcl_pt.b);
            ref.push_back(pcl_pt);
          }
        }
      }
    }
    ref.width = ref.points.size();
    ref.height = 1;
    ref.is_dense = true;
    octree->setInputCloud(tmp_cloud);
    octree->addPointsFromInputCloud();
  }

  // set point cloud
  octree->getVoxelCentroids(*oc_cloud);
  const AlignedPointXYZRGBNormalVector &oc = *oc_cloud;
  ref.resize(oc.size());
  for (unsigned i = 0; i < oc.size(); i++)
    ref.points[i] = oc[i];
  ref.height = 1;
  ref.width = ref.points.size();
  ref.is_dense = true;

  // filter largest custer
  if (param.filter_largest_cluster)
    filterCluster(tmp_cloud, cloud);
  else
    pcl::copyPointCloud(*tmp_cloud, cloud);
}

/**
 * @brief TSFGlobalCloudFilteringSimple::getGlobalCloudFiltered
 * @param frames
 * @param cloud
 */
void TSFGlobalCloudFilteringSimple::getGlobalCloudFiltered(const std::vector<v4r::TSFFrame::Ptr> &frames,
                                                           pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud) {
  // pcl::ScopeTime mt("[TSFGlobalCloudFilteringSimple::getGlobalCloudFiltered]");
  cloud.clear();
  Eigen::Matrix4f inv_pose;
  Eigen::Vector3f pt;
  cv::Point2f im_pt;
  Eigen::Matrix3f R, inv_R;
  Eigen::Vector3f t, inv_t;
  pcl::PointXYZRGBNormal pcl_pt;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PointCloud<pcl::PointXYZRGBNormal> &ref = *tmp_cloud;
  cv::Mat_<unsigned char> mask;

  pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGBNormal, pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>::Ptr
      octree;
  typedef pcl::octree::OctreePointCloudVoxelCentroid<
      pcl::PointXYZRGBNormal,
      pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>::AlignedPointTVector
      AlignedPointXYZRGBNormalVector;
  boost::shared_ptr<AlignedPointXYZRGBNormalVector> oc_cloud;
  oc_cloud.reset(new AlignedPointXYZRGBNormalVector());
  octree.reset(new pcl::octree::OctreePointCloudVoxelCentroid<
               pcl::PointXYZRGBNormal, pcl::octree::OctreeVoxelCentroidContainerXYZRGBNormal<pcl::PointXYZRGBNormal>>(
      param.voxel_size));
  octree->setResolution(param.voxel_size);

  double cos_rad_thr_delta_angle = cos(param.thr_delta_angle * M_PI / 180.);

  for (unsigned i = 0; i < frames.size(); i++) {
    ref.clear();
    const v4r::TSFFrame &frame = *frames[i];
    R = frame.delta_cloud_rgb_pose.topLeftCorner<3, 3>();
    t = frame.delta_cloud_rgb_pose.block<3, 1>(0, 3);
    v4r::invPose(frame.pose * base_transform, inv_pose);
    inv_R = inv_pose.topLeftCorner<3, 3>();
    inv_t = inv_pose.block<3, 1>(0, 3);

    getMask(frame.sf_cloud, mask);

    for (int v = 0; v < frame.sf_cloud.rows; v++) {
      for (int u = 0; u < frame.sf_cloud.cols; u++) {
        if (mask(v, u) < 128)
          continue;
        const v4r::Surfel &sf = frame.sf_cloud(v, u);
        if (isnan(sf.pt[0]) || isnan(sf.n[0]))
          continue;
        pt = R * sf.pt + t;
        if (dist_coeffs.empty())
          v4r::projectPointToImage(&pt[0], &intrinsic(0, 0), &im_pt.x);
        else
          v4r::projectPointToImage(&pt[0], &intrinsic(0, 0), &dist_coeffs(0), &im_pt.x);
        if (im_pt.x >= 0 && im_pt.y >= 0 && im_pt.x < frame.sf_cloud.cols && im_pt.y < frame.sf_cloud.rows) {
          pcl_pt.getVector3fMap() = inv_R * sf.pt + inv_t;
          if (pcl_pt.x < bb_min[0] || pcl_pt.x > bb_max[0] || pcl_pt.y < bb_min[1] || pcl_pt.y > bb_max[1] ||
              pcl_pt.z < bb_min[2] || pcl_pt.z > bb_max[2])
            continue;
          if (sf.weight >= param.thr_weight && sf.n.dot(-sf.pt.normalized()) > cos_rad_thr_delta_angle) {
            pcl_pt.getNormalVector3fMap() = inv_R * sf.n;
            getInterpolatedRGB(frame.sf_cloud, im_pt, pcl_pt.r, pcl_pt.g, pcl_pt.b);
            ref.push_back(pcl_pt);
          }
        }
      }
    }
    ref.width = ref.points.size();
    ref.height = 1;
    ref.is_dense = true;
    octree->setInputCloud(tmp_cloud);
    octree->addPointsFromInputCloud();
  }

  // set point cloud
  octree->getVoxelCentroids(*oc_cloud);
  const AlignedPointXYZRGBNormalVector &oc = *oc_cloud;
  ref.resize(oc.size());
  for (unsigned i = 0; i < oc.size(); i++)
    ref.points[i] = oc[i];
  ref.height = 1;
  ref.width = ref.points.size();
  ref.is_dense = true;

  // filter largest custer
  if (param.filter_largest_cluster)
    filterCluster(tmp_cloud, cloud);
  else
    pcl::copyPointCloud(*tmp_cloud, cloud);
}

/**
 * @brief TSFGlobalCloudFilteringSimple::getMesh
 * @param cloud
 * @param mesh
 */
void TSFGlobalCloudFilteringSimple::getMesh(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud,
                                            pcl::PolygonMesh &mesh) {
  PoissonTriangulation poisson(param.poisson_depth, param.samples_per_node, param.crop_mesh);
  poisson.reconstruct(cloud, mesh);
}

///**
// * @brief TSFGlobalCloudFilteringSimple::textureMapping
// * @param frames
// * @param mesh
// * @param dir
// * @param tex_mesh
// */
// void TSFGlobalCloudFilteringSimple::textureMapping(const std::vector< v4r::TSFFrame::Ptr > &frames, const
// pcl::PolygonMesh &mesh, const std::string &dir, pcl::TextureMesh &tex_mesh)
//{
//  if (frames.size()==0)
//    return;

//  cv::Mat_<cv::Vec3b> image;
//  pcl::PointCloud<pcl::PointXYZRGB> im_cloud;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

//  // Create the texturemesh object that will contain our UV-mapped mesh
//  tex_mesh.cloud = mesh.cloud;
//  std::vector< pcl::Vertices> polygon_1;

//  // push faces into the texturemesh object
//  polygon_1.resize (mesh.polygons.size ());
//  for(size_t i =0; i < mesh.polygons.size (); ++i)
//  {
//    polygon_1[i] = mesh.polygons[i];
//  }
//  tex_mesh.tex_polygons.push_back(polygon_1);
//  PCL_INFO ("\tInput mesh contains %d faces and %d vertices\n", tex_mesh.tex_polygons[0].size (), cloud->points.size
//  ());
//  PCL_INFO ("...Done.\n");

//  // Load textures and cameras poses and intrinsics
//  PCL_INFO ("\nLoading textures and camera poses...\n");
//  pcl::texture_mapping::CameraVector my_cams;

//  boost::filesystem::create_directories( dir );
//  std::string image_names = dir + "/tex_%08d.png";
//  char filename[PATH_MAX];

//  Eigen::Matrix4f pose, inv_pose;
//  for (unsigned i=0; i<frames.size(); i++)
//  {
//    // create cameras
//    v4r::TSFData::convert(frames[i]->sf_cloud, im_cloud);
//    v4r::convertImage(im_cloud, image);
//    snprintf(filename,PATH_MAX, image_names.c_str(), i);
//    cv::imwrite(filename, image);
//    pcl::TextureMapping<pcl::PointXYZ>::Camera cam;
//    pose =  frames[i]->delta_cloud_rgb_pose*frames[i]->pose*base_transform;
//    invPose(pose, inv_pose);
//    cam.pose = inv_pose;
//    cam.texture_file = filename;
//    //cam.focal_length =  (intrinsic(0,0)+intrinsic(1,1))/2.;
//    cam.focal_length_h = intrinsic(1,1);
//    cam.focal_length_w = intrinsic(0,0);
//    cam.center_h = intrinsic(1,2);
//    cam.center_w = intrinsic(0,2);
//    cam.height = image.rows;
//    cam.width = image.cols;
//    my_cams.push_back (cam);
//  }

//  tex_mesh.tex_materials.resize (frames.size () + 1);
//  for (unsigned i=0; i<tex_mesh.tex_materials.size(); i++)
//  {
//    // create materials
//    pcl::TexMaterial mesh_material;
//    mesh_material.tex_Ka.r = 0.2f;
//    mesh_material.tex_Ka.g = 0.2f;
//    mesh_material.tex_Ka.b = 0.2f;
//    mesh_material.tex_Kd.r = 0.8f;
//    mesh_material.tex_Kd.g = 0.8f;
//    mesh_material.tex_Kd.b = 0.8f;
//    mesh_material.tex_Ks.r = 1.0f;
//    mesh_material.tex_Ks.g = 1.0f;
//    mesh_material.tex_Ks.b = 1.0f;
//    mesh_material.tex_d = 1.0f;
//    mesh_material.tex_Ns = 75.0f;
//    mesh_material.tex_illum = 2;
//    std::stringstream tex_name;
//    tex_name << "material_" << i;
//    tex_name >> mesh_material.tex_name;
//    mesh_material.tex_file =(i < my_cams.size () ? my_cams[i].texture_file : (dir+"/occluded.png") );
//    tex_mesh.tex_materials[i] = mesh_material;
//  }

//  PCL_INFO ("\tLoaded %d textures.\n", my_cams.size ());
//  PCL_INFO ("...Done.\n");

//  // Sort faces
//  PCL_INFO ("\nSorting faces by cameras...\n");
//  pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
//  tm.textureMeshwithMultipleCameras(tex_mesh, my_cams);

//  PCL_INFO ("Sorting faces by cameras done.\n");
//  for(int i = 0 ; i <= my_cams.size() ; ++i)
//  {
//    PCL_INFO ("\tSub mesh %d contains %d faces and %d UV coordinates.\n", i, tex_mesh.tex_polygons[i].size (),
//    tex_mesh.tex_coordinates[i].size ());
//  }

//  // compute normals for the mesh
//  PCL_INFO ("\nEstimating normals...\n");
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//  tree->setInputCloud (cloud);
//  n.setInputCloud (cloud);
//  n.setSearchMethod (tree);
//  n.setKSearch (20);
//  n.compute (*normals);

//  // Concatenate XYZ and normal fields
//  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//  PCL_INFO ("...Done.\n");

//  pcl::toPCLPointCloud2 (*cloud_with_normals, tex_mesh.cloud);
//}

///**
// * @brief TSFGlobalCloudFilteringSimple::saveOBJFile
// * @param file_name
// * @param tex_mesh
// * @return
// */
// int TSFGlobalCloudFilteringSimple::saveOBJFile (const std::string &file_name, const pcl::TextureMesh &tex_mesh,
// unsigned precision)
//{
//  if (tex_mesh.cloud.data.empty ())
//  {
//    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
//    return (-1);
//  }

//  // Open file
//  std::ofstream fs;
//  fs.precision (precision);
//  fs.open (file_name.c_str ());

//  // Define material file
//  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
//  // Strip path for "mtllib" command
//  std::string mtl_file_name_nopath = mtl_file_name;
//  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

//  /* Write 3D information */
//  // number of points
//  int nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
//  int point_size = tex_mesh.cloud.data.size () / nr_points;

//  // mesh size
//  int nr_meshes = tex_mesh.tex_polygons.size ();
//  // number of faces for header
//  int nr_faces = 0;
//  for (int m = 0; m < nr_meshes; ++m)
//    nr_faces += tex_mesh.tex_polygons[m].size ();

//  // Write the header information
//  fs << "####" << std::endl;
//  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
//  fs << "# Vertices: " << nr_points << std::endl;
//  fs << "# Faces: " <<nr_faces << std::endl;
//  fs << "# Material information:" << std::endl;
//  fs << "mtllib " << mtl_file_name_nopath << std::endl;
//  fs << "####" << std::endl;

//  // Write vertex coordinates
//   fs << "# Vertices" << std::endl;
//   for (int i = 0; i < nr_points; ++i)
//   {
//     int xyz = 0;
//     // "v" just be written one
//     bool v_written = false;
//     for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
//     {
//       int count = tex_mesh.cloud.fields[d].count;
//       if (count == 0)
//         count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
//       int c = 0;
//       // adding vertex
//       if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
//                 tex_mesh.cloud.fields[d].name == "x" ||
//                 tex_mesh.cloud.fields[d].name == "y" ||
//                 tex_mesh.cloud.fields[d].name == "z"))
//       {
//         if (!v_written)
//         {
//             // write vertices beginning with v
//             fs << "v ";
//             v_written = true;
//         }
//         float value;
//         memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)],
//         sizeof (float));
//         fs << value;
//         if (++xyz == 3)
//             break;
//         fs << " ";
//       }
//     }
//     if (xyz != 3)
//     {
//       PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
//       return (-2);
//     }
//     fs << std::endl;
//   }
//   fs << "# "<< nr_points <<" vertices" << std::endl;

//   // Write vertex normals
//   for (int i = 0; i < nr_points; ++i)
//   {
//     int xyz = 0;
//     // "vn" just be written one
//     bool v_written = false;
//     for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
//     {
//       int count = tex_mesh.cloud.fields[d].count;
//       if (count == 0)
//       count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
//       int c = 0;
//       // adding vertex
//       if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
//       tex_mesh.cloud.fields[d].name == "normal_x" ||
//       tex_mesh.cloud.fields[d].name == "normal_y" ||
//       tex_mesh.cloud.fields[d].name == "normal_z"))
//       {
//         if (!v_written)
//         {
//           // write vertices beginning with vn
//           fs << "vn ";
//           v_written = true;
//         }
//         float value;
//         memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)],
//         sizeof (float));
//         fs << value;
//         if (++xyz == 3)
//           break;
//         fs << " ";
//       }
//     }
//     if (xyz != 3)
//     {
//     PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
//     return (-2);
//     }
//     fs << std::endl;
//   }

//   // Write vertex texture with "vt" (adding latter)
//   for (int m = 0; m < nr_meshes; ++m)
//   {
//     if(tex_mesh.tex_coordinates.size() == 0)
//       continue;

//     PCL_INFO ("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size (), m);
//     fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
//     for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
//     {
//       fs << "vt ";
//       fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
//     }
//   }

//   int f_idx = 0;

//   // int idx_vt =0;
//   PCL_INFO ("Writting faces...\n");
//   for (int m = 0; m < nr_meshes; ++m)
//   {
//     if (m > 0)
//       f_idx += tex_mesh.tex_polygons[m-1].size ();

//     if(tex_mesh.tex_materials.size() !=0)
//     {
//       fs << "# The material will be used for mesh " << m << std::endl;
//       //TODO pbl here with multi texture and unseen faces
//       fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
//       fs << "# Faces" << std::endl;
//     }
//     for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
//     {
//       // Write faces with "f"
//       fs << "f";
//       size_t j = 0;
//       // There's one UV per vertex per face, i.e., the same vertex can have
//       // different UV depending on the face.
//       for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
//       {
//         unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
//         fs << " " << idx
//         << "/" << 3*(i+f_idx) +j+1
//         << "/" << idx; // vertex index in obj file format starting with 1
//       }
//       fs << std::endl;
//     }
//     PCL_INFO ("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size () , m);
//     fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
//   }
//   fs << "# End of File";

//   // Close obj file
//   PCL_INFO ("Closing obj file\n");
//   fs.close ();
//   /* Write material defination for OBJ file*/
//   // Open file
//   PCL_INFO ("Writing material files\n");
//   //dont do it if no material to write
//   if(tex_mesh.tex_materials.size() ==0)
//     return (0);

//   std::ofstream m_fs;
//   m_fs.precision (precision);
//   m_fs.open (mtl_file_name.c_str ());

//   // default
//   m_fs << "#" << std::endl;
//   m_fs << "# Wavefront material file" << std::endl;
//   m_fs << "#" << std::endl;
//   for(int m = 0; m < nr_meshes; ++m)
//   {
//     m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
//     m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " <<
//     tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
//     m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " <<
//     tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
//     m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " <<
//     tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b).
//     This color shows up in highlights.
//     m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be
//     alpha.
//     m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
//     m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by
//     the material.
//     // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
//     // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
//     m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
//     m_fs << "###" << std::endl;
//   }
//   m_fs.close ();
//   return (0);
//}

/**
 * @brief TSFGlobalCloudFilteringSimple::setBaseTransform
 * @param transform
 */
void TSFGlobalCloudFilteringSimple::setBaseTransform(const Eigen::Matrix4f &transform) {
  base_transform = transform;
}

/**
 * @brief TSFGlobalCloudFilteringSimple::setROI
 * @param bb_lowerleft
 * @param bb_upper_right
 */
void TSFGlobalCloudFilteringSimple::setROI(const Eigen::Vector3f &bb_lowerleft, const Eigen::Vector3f &bb_upperright) {
  bb_min = bb_lowerleft;
  bb_max = bb_upperright;
}

/**
 * @brief TSFGlobalCloudFilteringSimple::setParameter
 * @param p
 */
void TSFGlobalCloudFilteringSimple::setParameter(const Parameter &p) {
  param = p;
}

/**
 * @brief TSFGlobalCloudFilteringSimple::setCameraParameter
 * @param _intrinsic
 * @param _dist_coeffs
 */
void TSFGlobalCloudFilteringSimple::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) {
  dist_coeffs = cv::Mat_<double>();
  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else
    intrinsic = _intrinsic;
  if (!_dist_coeffs.empty()) {
    dist_coeffs = cv::Mat_<double>::zeros(1, 8);
    for (int i = 0; i < _dist_coeffs.cols * _dist_coeffs.rows; i++)
      dist_coeffs(0, i) = _dist_coeffs.at<double>(0, i);
  }
}
}

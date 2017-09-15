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

#ifndef Q_MOC_RUN
#include "ObjectSegmentation.h"

#include <cmath>
#include <boost/filesystem.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/keypoints/impl/PoseIO.hpp>
#include <v4r/common/convertCloud.h>
#include <v4r/common/convertNormals.h>
#include <v4r/common/convertImage.h>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <v4r/common/noise_models.h>
#include <v4r/registration/noise_model_based_cloud_integration.h>
#include <v4r/registration/MvLMIcp.h>
#include <pcl/segmentation/extract_clusters.h>
#endif

using namespace std;


/**
 * @brief ObjectSegmentation::ObjectSegmentation
 */
ObjectSegmentation::ObjectSegmentation()
 : cmd(UNDEF), m_run(false), create_cloud(true), create_views(true), create_mesh(true), create_tracking_model(true),
   voxel_size(0.001),poisson_depth(7), poisson_samples(2), max_dist(0.01f), max_iterations(50), diff_type(1), use_mvicp(true), use_noise(false),
   filter_largest_cluster(true), edge_radius_px(5), max_point_dist(0.03),
   bb_min(Eigen::Vector3f(-FLT_MAX,-FLT_MAX,-FLT_MAX)), bb_max(Eigen::Vector3f(FLT_MAX,FLT_MAX,FLT_MAX)),
   object_base_transform(Eigen::Matrix4f::Identity()), seg_offs(0.01)
{
  tmp_cloud0.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  tmp_cloud1.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  tmp_cloud2.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  ncloud_filt.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

  intrinsic = cv::Mat_<double>::eye(3,3);
  intrinsic(0,0) = cam_params.f[0]; intrinsic(1,1) = cam_params.f[1];
  intrinsic(0,2) = cam_params.c[0]; intrinsic(1,2) = cam_params.c[1];

  ba_param.depth_error_scale = 100;
  ba_param.px_error_scale = 1;
  ba_param.optimize_delta_cloud_rgb_pose_global = true;
  ba_param.optimize_delta_cloud_rgb_pose = false;
  ba_param.optimize_focal_length = false;
  ba_param.optimize_principal_point = false;
  ba_param.optimize_radial_k1 = false;
  ba_param.optimize_radial_k2 = false;
  ba_param.optimize_radial_k3 = false;
  ba_param.optimize_tangential_p1 = false;
  ba_param.optimize_tangential_p2 = false;

  ba.setParameter(ba_param);
  ba.setCameraParameter(intrinsic, dist_coeffs);
  dist_coeffs.copyTo(dist_coeffs_opti);
  intrinsic.copyTo(intrinsic_opti);

}

/**
 * @brief ObjectSegmentation::~ObjectSegmentation
 */
ObjectSegmentation::~ObjectSegmentation()
{

}



/******************************** public *******************************/

/**
 * @brief ObjectSegmentation::start
 * @param cam_id
 */
void ObjectSegmentation::start()
{
  QThread::start();
}

/**
 * @brief ObjectSegmentation::stop
 */
void ObjectSegmentation::stop()
{
  if(m_run)
  {
    m_run = false;
    this->wait();
  }
}

/**
 * @brief ObjectSegmentation::isRunning
 * @return
 */
bool ObjectSegmentation::isRunning()
{
  return m_run;
}


/**
 * @brief ObjectSegmentation::finishedSegmentation
 */
void ObjectSegmentation::finishModelling()
{
  cmd = FINISH_OBJECT_MODELLING;
  start();
}


/**
 * @brief ObjectSegmentation::savePointClouds
 * @param _folder
 * @param _modelname
 * @return
 */
bool ObjectSegmentation::savePointClouds(const std::string &_folder, const std::string &_modelname)
{
  if (clouds.size()==0)
    return false;

  char filename[PATH_MAX];
  boost::filesystem::create_directories(_folder + "/models/" + _modelname + "/views" );

  // store global model
  if (create_cloud && ncloud_filt->points.size()>0)
    pcl::io::savePCDFileBinary(_folder + "/models/" + _modelname + "/3D_model.pcd", *ncloud_filt);
  if (create_mesh && mesh.polygons.size()>0)
    pcl::io::savePLYFile(_folder + "/models/" + _modelname + "/mesh.ply", mesh);

  boost::filesystem::create_directories(_folder + "/models/" + _modelname + "/views" );

  std::string cloud_names = _folder + "/models/" + _modelname + "/views/cloud_%08d.pcd";
  std::string image_names = _folder + "/models/" + _modelname + "/views/image_%08d.jpg";
  std::string pose_names = _folder + "/models/" + _modelname + "/views/pose_%08d.txt";
  std::string mask_names = _folder + "/models/" + _modelname + "/views/mask_%08d.png";
  std::string idx_names = _folder + "/models/" + _modelname + "/views/object_indices_%08d.txt";


  if (create_views)
  {
    for (unsigned i=0; i<clouds.size(); i++)
    {
      if (indices[i].empty()) continue;

      // store indices
      snprintf(filename, PATH_MAX, idx_names.c_str(), i);
      std::ofstream mask_f (filename);
      for(unsigned j=0; j < indices[i].size(); j++)
        mask_f << indices[i][j] << std::endl;
      mask_f.close();

      // store cloud
      snprintf(filename,PATH_MAX, cloud_names.c_str(), i);
      pcl::io::savePCDFileBinary(filename, *clouds[i]);

      // store image
      v4r::convertImage(*clouds[i], image);
      snprintf(filename,PATH_MAX, image_names.c_str(), i);
      cv::imwrite(filename, image);

      // store poses
      snprintf(filename,PATH_MAX, pose_names.c_str(), i);
      v4r::writePose(filename, std::string(), inv_poses[i]);

      // store masks
      snprintf(filename,PATH_MAX, mask_names.c_str(), i);
      cv::imwrite(filename, masks[i]);
    }
  }

  // store tracking model
  if (create_tracking_model)
    tm.saveTrackingModel(_folder, _modelname);

  return true;
}

void ObjectSegmentation::set_roi_params(const double &_bbox_scale_xy, const double &_bbox_scale_height, const double &_seg_offs)
{
  seg_offs = _seg_offs;
}

/**
 * @brief ObjectSegmentation::setData
 * @param _cameras
 * @param _clouds
 */
void ObjectSegmentation::setData(const std::vector<v4r::TSFFrame::Ptr> &_map_frames, const Eigen::Matrix4f &_base_transform, const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max)
{
  map_frames = _map_frames;
  object_base_transform = _base_transform;
  bb_min = _bb_min;
  bb_max = _bb_max;
}

/**
 * @brief ObjectSegmentation::setCameraParameter
 * @param _intrinsic
 * @param _dist_coeffs
 */
void ObjectSegmentation::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  if (_intrinsic.empty())
    return;
  _intrinsic.copyTo(intrinsic);
  _dist_coeffs.copyTo(dist_coeffs);
  ba.setCameraParameter(intrinsic, dist_coeffs);
  dist_coeffs.copyTo(dist_coeffs_opti);
  intrinsic.copyTo(intrinsic_opti);
}




/*********************************** private *******************************************/
/**
 * @brief ObjectSegmentation::run
 * main loop
 */
void ObjectSegmentation::run()
{
  m_run=true;

  switch (cmd)
  {
  case FINISH_OBJECT_MODELLING:
  {
    if (map_frames.size()>0)
    {
      ba.optimize(map_frames);
      getSegmentedViews();
      if (use_mvicp) optimizePosesMultiviewICP();
      createCloudModel();
      if (create_tracking_model)
      {
        tm.setCameraParameter(intrinsic_opti, dist_coeffs_opti);
        tm.createTrackingModel(map_frames, masks, object_base_transform);
      }

      emit printStatus(std::string("Status: Finised object modelling"));
      emit update_model_cloud(ncloud_filt);
      emit update_visualization();
    }
    else
    {
      emit printStatus(std::string("Status: No data for object modelling available!"));
    }

    emit finishedModelling();
    break;
  }
  default:
    break;
  }

  cmd = UNDEF;
  m_run=false;
}

/**
 * @brief ObjectSegmentation::createMaskFromROI
 * @param cloud
 * @param mask
 * @param object_base_transform
 * @param bb_min
 * @param bb_max
 * @param roi_offs
 */
void ObjectSegmentation::createMaskFromROI(const v4r::DataMatrix2D<Eigen::Vector3f> &cloud, cv::Mat_<unsigned char> &mask, const Eigen::Matrix4f &_object_base_transform, const Eigen::Vector3f &_bb_min, const Eigen::Vector3f &_bb_max, const double &_roi_offs)
{
  Eigen::Vector3f pt;
  Eigen::Matrix4f inv_pose;

  v4r::invPose(_object_base_transform,inv_pose);

  Eigen::Matrix3f R = inv_pose.topLeftCorner<3,3>();
  Eigen::Vector3f t = inv_pose.block<3,1>(0,3);

  mask = cv::Mat_<unsigned char>::zeros(cloud.rows, cloud.cols);

  for (unsigned i=0; i<cloud.data.size(); i++)
  {
    const Eigen::Vector3f &pt0 = cloud.data[i];

    if (!std::isnan(pt0[0]) && !std::isnan(pt0[1]) && !std::isnan(pt0[2]))
    {
      pt = R*pt0 + t;

      if (pt[0]>_bb_min[0] && pt[0]<_bb_max[0] && pt[1]>_bb_min[1] && pt[1]<_bb_max[1] && pt[2]>_roi_offs && pt[2]<_bb_max[2])
        mask(i) = 255;
    }
  }
}

/**
 * @brief ObjectSegmentation::getMaskFromBasePlane
 * @param cloud
 * @param pose
 * @param mask
 */
void ObjectSegmentation::getMaskFromBasePlane(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Matrix4f &pose, cv::Mat_<unsigned char> &mask, std::vector<int> &indices)
{
  indices.clear();
  if (bb_min[0]==-FLT_MAX && bb_max[0]==FLT_MAX && bb_min[1]==-FLT_MAX && bb_max[1]==FLT_MAX && bb_min[2]==-FLT_MAX && bb_max[2]==FLT_MAX)
  {
    mask = cv::Mat_<unsigned char>::ones(cloud.height,cloud.width)*255;
    for (int i=0; i<mask.cols*mask.rows; i++)
      if (mask(i)>128) indices.push_back(i);
    return;
  }

  mask = cv::Mat_<unsigned char>::zeros(cloud.height,cloud.width);

  Eigen::Matrix4f inv_pose;
  v4r::invPose(pose, inv_pose);
  std::vector<int> inds;
  pcl::PointCloud<pcl::PointXYZRGB> &rc0 = *tmp_cloud0;
  pcl::PointCloud<pcl::PointXYZRGB> &rc1 = *tmp_cloud1;
  pcl::PointCloud<pcl::PointXYZRGB> &rc2 = *tmp_cloud2;

  pcl::transformPointCloud(cloud, rc0, inv_pose);
  rc1 = rc0;

  for (unsigned i=0; i<rc1.points.size(); i++)
  {
    pcl::PointXYZRGB &pt = rc1.points[i];
    if (!std::isnan(pt.z))
    {
      if (fabs(pt.z)>seg_offs)
        pt.getVector3fMap() = Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::quiet_NaN());
    }
  }

  pcl::removeNaNFromPointCloud (rc1, rc2, inds);

  double a,b,c,d;
  if (rc2.points.size()>3)
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (tmp_cloud2);
    seg.segment(*inliers, *coefficients);

    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];
  }

  for (unsigned i=0; i<rc0.points.size(); i++)
  {
    pcl::PointXYZRGB &pt = rc0.points[i];
    if (!std::isnan(pt.z))
    {
      if (rc2.points.size()<=3 || pcl::pointToPlaneDistanceSigned(pt,a,b,c,d)>seg_offs)
        mask(i) = 255;
    }
  }

  cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*param.morph_size + 1, 2*param.morph_size+1 ), cv::Point( param.morph_size, param.morph_size ) );
  cv::morphologyEx(mask, tmp_mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(tmp_mask, mask, cv::MORPH_CLOSE, element);

  for (int i=0; i<mask.cols*mask.rows; i++)
    if (mask(i)>128) indices.push_back(i);
}


void ObjectSegmentation::getSegmentedViews()
{
  clouds.resize(map_frames.size());
  inv_poses.resize(map_frames.size());
  masks.resize(map_frames.size());
  indices.resize(map_frames.size());

  for (unsigned i=0; i<map_frames.size(); i++)
  {
    clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    v4r::TSFData::convert(map_frames[i]->sf_cloud, *clouds[i]);
    v4r::invPose(map_frames[i]->pose*object_base_transform, inv_poses[i]);
    getMaskFromBasePlane(*clouds[i], map_frames[i]->pose*object_base_transform, masks[i], indices[i]);
  }
}

void ObjectSegmentation::createObjectCloudFilteredNguyen()
{
  if (clouds.size()==0 || masks.size()!=clouds.size())
    return;

  v4r::NguyenNoiseModelParameter nmparam;
  v4r::NguyenNoiseModel<pcl::PointXYZRGB> nm(nmparam);
  std::vector< std::vector<std::vector<float> > > pt_properties (clouds.size());
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr > ptr_clouds(clouds.size());
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ncloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>() );
  pcl::PointCloud<pcl::Normal>::Ptr ns;

  if (clouds.size()>0)
  {
    normals.resize(clouds.size());
    ptr_clouds.resize(clouds.size());

    for (unsigned i=0; i<clouds.size(); i++)
    {
      ptr_clouds[i] = clouds[i];
      ns.reset((new pcl::PointCloud<pcl::Normal>()));
      v4r::TSFData::convert(map_frames[i]->sf_cloud, *ns);
      normals[i] = ns;
      nm.setInputCloud(clouds[i]);
      nm.setInputNormals(normals[i]);
      nm.compute();
      pt_properties[i] = nm.getPointProperties();
    }

    v4r::NMBasedCloudIntegrationParameter _nmparam;
    _nmparam.octree_resolution_ = voxel_size; //0.001
    _nmparam.min_px_distance_to_depth_discontinuity_ = edge_radius_px; //3
    _nmparam.min_points_per_voxel_ = 1;
    tmp_cloud0.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    v4r::NMBasedCloudIntegration<pcl::PointXYZRGB> nmIntegration(_nmparam);
    nmIntegration.setInputClouds(ptr_clouds);
    nmIntegration.setTransformations(inv_poses);
    nmIntegration.setInputNormals(normals);
    nmIntegration.setIndices(indices);
    nmIntegration.setPointProperties(pt_properties);
    nmIntegration.compute(tmp_cloud0);
    big_normals.reset(new pcl::PointCloud<pcl::Normal>);
    nmIntegration.getOutputNormals(big_normals);

    // create model cloud with normals
    ncloud_filt.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    ncloud.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::concatenateFields(*tmp_cloud0, *big_normals, *ncloud);

    // filter ec
    if (filter_largest_cluster)
    {
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      tree->setInputCloud (ncloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
      ec.setClusterTolerance (max_point_dist);
      ec.setMinClusterSize (50);
      ec.setSearchMethod (tree);
      ec.setInputCloud (ncloud);
      ec.extract (cluster_indices);

      int cnt_max = 0;
      std::vector<pcl::PointIndices>::const_iterator it_max = cluster_indices.end();

      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        if (it->indices.size()>(unsigned)cnt_max)
        {
          it_max = it;
          cnt_max = it->indices.size();
        }
      }

      if (it_max!=cluster_indices.end())
      {
        pcl::copyPointCloud(*ncloud, it_max->indices, *ncloud_filt);
      }
    }
    else
    {
      pcl::copyPointCloud(*ncloud, *ncloud_filt);
    }
  }
}


void ObjectSegmentation::optimizePosesMultiviewICP()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_filtered(clouds.size());

  for (unsigned i=0; i<clouds.size(); i++)
  {
    clouds_filtered[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*clouds[i], indices[i], *cloud_segmented);
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud_segmented);
    //filter.setInputCloud(clouds[i]);
    filter.setDownsampleAllData(true);
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter.filter(*clouds_filtered[i]);
  }

  v4r::Registration::MvLMIcp<pcl::PointXYZRGB> nl_icp;
  nl_icp.setInputClouds(clouds_filtered);
  nl_icp.setPoses(inv_poses);
  nl_icp.setMaxCorrespondenceDistance(max_dist);
  nl_icp.setMaxIterations(max_iterations);
  nl_icp.setDiffType(diff_type);
  nl_icp.compute();
  inv_poses = nl_icp.getFinalPoses();

  //  v4r::invPose(map_frames[i]->pose*object_base_transform, inv_poses[i]);
  Eigen::Matrix4f pose, inv_object_base;
  v4r::invPose(object_base_transform, inv_object_base);
  for (unsigned i=0; i<map_frames.size(); i++)
  {
    v4r::invPose(inv_poses[i], pose);
    map_frames[i]->pose = pose*inv_object_base;
    map_frames[i]->delta_cloud_rgb_pose.setIdentity();
  }
}

void ObjectSegmentation::createCloudModel()
{
  if (create_cloud || create_mesh)
  {
    // optimize map
    ba.getCameraParameter(intrinsic_opti, dist_coeffs_opti);

    // create model in global coordinates
    cout<<"Create pointcloud model..."<<endl;
    v4r::TSFGlobalCloudFilteringSimple gfilt;
    v4r::TSFGlobalCloudFilteringSimple::Parameter filt_param;
    filt_param.filter_largest_cluster = filter_largest_cluster;
    filt_param.voxel_size = voxel_size;
    filt_param.thr_weight = 2;
    filt_param.thr_delta_angle = 80.;
    filt_param.poisson_depth = poisson_depth;
    filt_param.samples_per_node = poisson_samples;
    gfilt.setParameter(filt_param);

    gfilt.setCameraParameter(intrinsic_opti, dist_coeffs_opti);
    gfilt.setBaseTransform(object_base_transform);
    gfilt.setROI(bb_min, bb_max);

    if (use_noise)
    {
      createObjectCloudFilteredNguyen();
    }
    else
    {
      gfilt.getGlobalCloudMasked(map_frames, masks, *ncloud_filt);
    }

    cout<<"Creat mesh..."<<endl;
    if (create_mesh)
    {
      gfilt.getMesh(ncloud_filt, mesh);
//      cout<<"[ObjectSegmentation::createCloudModel] TODO: - bug inv poses, - store tex_mesh, - tmp-folder"<<endl;
//      gfilt.textureMapping(map_frames, mesh, std::string("data")+std::string("/tmp/"), tex_mesh);
//      gfilt.saveOBJFile("tex_mesh.obj", tex_mesh, 5);
    }

    cout<<"Finished!"<<endl;
  }
}


/**
 * @brief ObjectSegmentation::convertImage
 * @param cloud
 * @param image
 */
void ObjectSegmentation::convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat_<cv::Vec3b> &_image)
{
  _image = cv::Mat_<cv::Vec3b>(cloud.height, cloud.width);

  for (unsigned v = 0; v < cloud.height; v++)
  {
    for (unsigned u = 0; u < cloud.width; u++)
    {
      cv::Vec3b &cv_pt = _image.at<cv::Vec3b> (v, u);
      const pcl::PointXYZRGB &pt = cloud(u,v);

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}




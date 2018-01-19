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


#include <v4r/camera_tracking_and_mapping/OptimizeBundleColours.hh>
//#include <v4r/camera_tracking_and_mapping/BACostFunctions.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/reconstruction/impl/projectPointToImage.hpp>
#include <v4r/camera_tracking_and_mapping/TSFData.h>
#include <v4r/common/convertImage.h>
#include <stdlib.h>
#include <time.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace v4r
{


using namespace std;


class ColourCorrespondenceError
{
public:
  ColourCorrespondenceError(const cv::Vec3f &_col0, const cv::Vec3f &_col1)
  : col0(_col0), col1(_col1) {}

  /**
   */
  template <typename T>
  bool operator()(const T* const col_scale0,
                  const T* const col_scale1,
                  T* residuals) const {

    residuals[0] = col_scale0[0]*T(col0[0])-col_scale1[0]*T(col1[0]);
    residuals[1] = col_scale0[1]*T(col0[1])-col_scale1[1]*T(col1[1]);
    residuals[2] = col_scale0[2]*T(col0[2])-col_scale1[2]*T(col1[2]);

    return true;
  }

  const cv::Vec3f &col0;
  const cv::Vec3f &col1;
};


class ColourScaleNormError
{
public:
  ColourScaleNormError(const int &_nb_scales, const std::vector< std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > > &_rgb_correspondences)
  : nb_parameters(_nb_scales), rgb_correspondences(_rgb_correspondences) {}

  /**
   */
  template <typename T>
  bool operator()(T const* const* parameters,
                  T* residuals) const {

    const T* x = parameters[0];

    unsigned z=0;
    for (unsigned i=0; i<rgb_correspondences.size(); i++)
    {
      for (unsigned j=0; j<rgb_correspondences[i].size(); j++)
      {
        const v4r::triple<cv::Vec3f, int, cv::Vec3f> &c = rgb_correspondences[i][j];
        residuals[z+0] = x[3*i+0]*T(c.first[0])-x[3*c.second+0]*T(c.third[0]);
        residuals[z+1] = x[3*i+1]*T(c.first[1])-x[3*c.second+1]*T(c.third[1]);
        residuals[z+2] = x[3*i+2]*T(c.first[2])-x[3*c.second+2]*T(c.third[2]);
        z+=3;
      }
    }


    T sum = T(0);
    for (int i=0; i<nb_parameters; i++)
      sum = sum + x[i];

    residuals[z] = T(10) * (T(nb_parameters) - sum);

    return true;
  }

  const int &nb_parameters;
  const std::vector< std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > > &rgb_correspondences;
};


void TukeyLoss::Evaluate(double s, double* rho) const {
  if (s <= a_squared_) {
    // Inlier region.
    const double value = 1.0 - s / a_squared_;
    const double value_sq = value * value;
    rho[0] = a_squared_ / 6.0 * (1.0 - value_sq * value);
    rho[1] = 0.5 * value_sq;
    rho[2] = -1.0 / a_squared_ * value;
  } else {
    // Outlier region.
    rho[0] = a_squared_ / 6.0;
    rho[1] = 0.0;
    rho[2] = 0.0;
  }
}



/************************************************************************************
 * Constructor/Destructor
 */
OptimizeBundleColours::OptimizeBundleColours(const Parameter &p)
{
  setParameter(p);
}


OptimizeBundleColours::~OptimizeBundleColours()
{
}

/**
 * @brief OptimizeBundleColours::getLinkedFrames
 */
void OptimizeBundleColours::getLinkedFrames(const unsigned &idx, const std::vector< std::vector< v4r::triple<int, cv::Point2f, Eigen::Vector3f > > >  &projections, std::set<int> &links)
{
  links.clear();

  for (unsigned i=0; i<projections.size(); i++)
  {
    for (unsigned j=0; j<projections[i].size(); j++)
      if (projections[i][j].first>(int)idx)
        links.insert(projections[i][j].first);
  }
}

/**
 * @brief OptimizeBundleColours::pickCol
 * @param pt
 * @param delta_pose
 * @param im_lin
 * @param col0
 * @return
 */
bool OptimizeBundleColours::pickCol(const Eigen::Vector3f &pt, const Eigen::Matrix4f &delta_pose, const cv::Mat &im_lin, cv::Point2f &_im_pt, cv::Vec3f &col)
{
  Eigen::Vector3f pt1 = delta_pose.topLeftCorner<3, 3>()*pt + delta_pose.block<3,1>(0, 3);
  cv::Point2f im_pt;

  if (dist_coeffs.empty())
    projectPointToImage(&pt1[0], &intrinsic(0,0), &im_pt.x);
  else projectPointToImage(&pt1[0], &intrinsic(0,0), &dist_coeffs(0,0), &im_pt.x);

  _im_pt = im_pt;

  if ((int)im_pt.x>=0 && (int)im_pt.x<im_lin.cols && (int)im_pt.y>=0 && (int)im_pt.y<im_lin.rows)
  {
    col = im_lin.at<cv::Vec3f>(im_pt.y,im_pt.x);
    return true;
  }
  return false;
}

/**
 * @brief OptimizeBundleColours::pickCol
 * @param pt
 * @param delta_pose
 * @param sf_cloud
 * @param im_lin
 * @param _im_pt
 * @param col
 * @return
 */
bool OptimizeBundleColours::pickCol(const Eigen::Vector3f &pt, const Eigen::Matrix4f &delta_pose, const v4r::DataMatrix2D<Surfel> &sf_cloud, const cv::Mat &im_lin, cv::Point2f &_im_pt, cv::Vec3f &col)
{
  Eigen::Vector3f pt1 = delta_pose.topLeftCorner<3, 3>()*pt + delta_pose.block<3,1>(0, 3);
  cv::Point2f im_pt;

  if (dist_coeffs.empty())
    projectPointToImage(&pt1[0], &intrinsic(0,0), &im_pt.x);
  else projectPointToImage(&pt1[0], &intrinsic(0,0), &dist_coeffs(0,0), &im_pt.x);

  _im_pt = im_pt;

  if ((int)im_pt.x>=0 && (int)im_pt.x<im_lin.cols && (int)im_pt.y>=0 && (int)im_pt.y<im_lin.rows && !isnan(sf_cloud(im_pt.y,im_pt.x).pt[0]) && fabs(1./pt[2]-1./sf_cloud(im_pt.y,im_pt.x).pt[2])<param.inv_inl_dist)
  {
    col = im_lin.at<cv::Vec3f>(im_pt.y,im_pt.x);
    return true;
  }
  return false;
}

/**
 * @brief OptimizeBundleColours::sampelColourCorrespondences
 * @param map
 * @param lin_images
 * @param idx0
 * @param idx1
 * @param corrs
 */
//double sum=0;
//unsigned nb = 0;
void OptimizeBundleColours::sampelColourCorrespondences(const std::vector<TSFFrame::Ptr> &map, const std::vector<cv::Mat> &_lin_images, int idx0, int idx1, std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > &corrs)
{
  const TSFFrame &frame0 = *map[idx0];
  const TSFFrame &frame1 = *map[idx1];
  const cv::Mat &im_lin0 = _lin_images[idx0];
  const cv::Mat &im_lin1 = _lin_images[idx1];
  const cv::Mat_<unsigned char> &eds = edges[idx0];

  cv::Vec3f col0, col1;
  Eigen::Matrix4f inv_pose0, pose_im01;
  cv::Point2f im_pt0, im_pt1;

  invPose(frame0.pose, inv_pose0);
  pose_im01 = frame1.delta_cloud_rgb_pose * frame1.pose * inv_pose0;

//  cv::Mat_<cv::Vec3b> dbg0, dbg1;
//  TSFData::convert(frame0.sf_cloud, dbg0);
//  TSFData::convert(frame1.sf_cloud, dbg1);


  unsigned z=0;
  int cnt=0;
  int x,y;

  mask = cv::Mat_<unsigned char>::zeros(frame0.sf_cloud.rows, frame0.sf_cloud.cols);

  // index valid points
  for (int v=0; v<frame0.sf_cloud.rows; v++)
    for (int u=0; u<frame0.sf_cloud.cols; u++)
      if (!std::isnan(frame0.sf_cloud(v,u).pt[0]) && eds(v,u)<param.grad_threshold)
        mask(v,u) = 255;

  // random sampling of corresponding colour points
  while(z<param.max_sampling_trials && cnt<param.nb_col_samples_per_linked_frame)
  {
    x = rand()%frame0.sf_cloud.cols;
    y = rand()%frame0.sf_cloud.rows;

    if (x>=0 && x<frame0.sf_cloud.cols && y>=0 && y<frame0.sf_cloud.rows && mask(y,x))
    {
      const Surfel &s0 = frame0.sf_cloud(y,x);

      if (pickCol(s0.pt, frame0.delta_cloud_rgb_pose, im_lin0, im_pt0, col0) && pickCol(s0.pt, pose_im01, frame1.sf_cloud, im_lin1, im_pt1, col1))
      {
        const Surfel &s1 = frame1.sf_cloud(im_pt1.y, im_pt1.x);

        if (s0.r>param.saturation_offs_low && s0.r<param.saturation_offs_high &&
            s0.g>param.saturation_offs_low && s0.g<param.saturation_offs_high &&
            s0.b>param.saturation_offs_low && s0.b<param.saturation_offs_high &&
            s1.r>param.saturation_offs_low && s1.r<param.saturation_offs_high &&
            s1.g>param.saturation_offs_low && s1.g<param.saturation_offs_high &&
            s1.b>param.saturation_offs_low && s1.b<param.saturation_offs_high)
        {
//        cv::Vec3b col(rand()%255,rand()%255, rand()%255);
//        cv::circle(dbg0, im_pt0, 2, col, 1);
//        cv::circle(dbg1, im_pt1, 2, col, 1);
        //cout<<col0[0]<<","<<col0[1]<<","<<col0[2]<<"="<<col1[0]<<","<<col1[1]<<","<<col1[2]<<" | ";
        corrs.push_back(v4r::triple<cv::Vec3f, int, cv::Vec3f>(col0, idx1, col1));
        mask(y,x) = 0;
        cnt++;
//        sum += fabs(col0[0]-col1[0]);
//        sum += fabs(col0[1]-col1[1]);
//        sum += fabs(col0[2]-col1[2]);
//        nb+=3;
        }
      }
    }

    z++;
  }

//  cout<<"("<<cnt<<"/"<<param.nb_col_samples_per_linked_frame<<"/"<<z<<") ";
//  cout<<"mean: "<<sum/(double)nb<<endl;
//  cv::imshow("dbg0", dbg0);
//  cv::imshow("dbg1", dbg1);
//  cv::waitKey(0);
}

/**
 * @brief OptimizeBundleColours::sampleCorrespondences
 * @param map
 * @param lin_images
 *
 * std::vector< std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > > rgb_correspondences;
 */
void OptimizeBundleColours::sampleCorrespondences(const std::vector<TSFFrame::Ptr> &map, const std::vector<cv::Mat> &_lin_images)
{
  std::set<int> links;
  std::set<int>::iterator it;
  srand (time(NULL));

  rgb_correspondences.assign(map.size(), std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> >() );

  for (unsigned i=0; i<map.size(); i++)
  {
    links.clear();
    if (map[i]->fw_link!=-1) links.insert(map[i]->fw_link);
    //if (map[i]->bw_link!=-1) links.insert(map[i]->bw_link);

    for (unsigned j=0; j<map[i]->loop_links.size(); j++)
      links.insert(map[i]->loop_links[j]);

    if (param.use_projection_links) getLinkedFrames(i, map[i]->projections, links);  // optional

    for (it=links.begin(); it!=links.end(); it++)
    {
      if ((int)i!=*it)
      {
        sampelColourCorrespondences(map, _lin_images, i, *it, rgb_correspondences[i]);
      }
    }
    cout<<"i="<<i<<", nb-correspondences="<<rgb_correspondences[i].size()<<endl;
  }
}

/**
 * @brief OptimizeBundleColours::optimizeColourScales
 */
void OptimizeBundleColours::optimizeColourScales(int idx)
{
  if ((int)rgb_correspondences.size()<=idx || rgb_correspondences[idx].size()<10)
  {
    cout<<"[OptimizeBundleColours::optimizeColourScales] No colour correspondences initialized!"<<endl;
    return;
  }

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // set up cost functions
  //for (unsigned i=0; i<rgb_correspondences.size(); i++)
  {
    double *c0 = &colour_scales_data[idx*3];
    const std::vector< v4r::triple<cv::Vec3f, int, cv::Vec3f> > &corrs = rgb_correspondences[idx];

    for (unsigned j=0; j<corrs.size(); j++)
    {
      double *c1 = &colour_scales_data[corrs[j].second*3];
      problem.AddResidualBlock( new ceres::AutoDiffCostFunction< ColourCorrespondenceError, 3, 3,3 >(
                                  new ColourCorrespondenceError(corrs[j].first, corrs[j].third) ),
                                (param.use_robust_loss?new ceres::CauchyLoss(param.loss_scale):NULL), c0, c1);
    }
  }

  problem.SetParameterBlockConstant(&colour_scales_data[idx*3]);

//  int nb_residuals = 0;
//  for (unsigned i=0; i<rgb_correspondences.size(); i++)
//    nb_residuals += rgb_correspondences[i].size();

//  const int stride = 4;
//  int nb_scales = colour_scales_data.size();
//  ceres::DynamicAutoDiffCostFunction<ColourScaleNormError, stride>* cost_function = new ceres::DynamicAutoDiffCostFunction<ColourScaleNormError, stride>(new ColourScaleNormError(nb_scales, rgb_correspondences));
//  cost_function->AddParameterBlock(nb_scales);
//  cost_function->SetNumResiduals(3*nb_residuals + 1);

//  std::vector<double*> blocks;
//  blocks.push_back(&colour_scales_data[0]);
//  problem.AddResidualBlock(cost_function, (param.use_robust_loss?new ceres::CauchyLoss(param.loss_scale):NULL), blocks);


//  std::vector<int> const_params(3);
//  const_params[0] = 0;
//  const_params[1] = 1;
//  const_params[2] = 2;

//  ceres::SubsetParameterization *subset_parameterization =
//        new ceres::SubsetParameterization(nb_scales, const_params);
//  problem.SetParameterization(&colour_scales_data[0], subset_parameterization);


  ceres::Solver::Options options;
//  options.use_nonmonotonic_steps = true;
//  options.preconditioner_type = ceres::SCHUR_JACOBI;
//  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
//  options.use_inner_iterations = true;
  options.max_num_iterations = 100;

  options.minimizer_progress_to_stdout = false;
//  options.minimizer_progress_to_stdout = true;

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  std::cout << "Final report:\n" << summary.FullReport();
}



/***************************************************************************************/

/**
 * @brief OptimizeBundleColours::optimize
 * @param map
 * @param lin_images
 */
void OptimizeBundleColours::optimize(const std::vector<TSFFrame::Ptr> &map)
{
  if (intrinsic.empty())
    throw std::runtime_error("[OptimizeBundleColours::optimize] camera intrinsics not set!");

  if (map.size()<2)
  {
    cout<<"[OptimizeBundleColours::optimize] Invalid data (e.g. map size = 0)"<<endl;
    return;
  }
  if (path_to_crf_file.size()==0)
  {
    cout<<"[OptimizeBundleColours::optimize] No camera response function calibration file available (crf-file)"<<endl;
    return;
  }

  // convert to liniar space
  cv::Mat image;
  radical::RadiometricResponse rr(path_to_crf_file);
  lin_images.resize(map.size());
  edges.resize(map.size());
  cv::Mat im_gray;
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  for (unsigned i=0; i<map.size(); i++)
  {
    // convert image
    v4r::TSFData::convert(map[i]->sf_cloud, image);
    rr.inverseMap(image, lin_images[i]);
    // detect edges
    cv::cvtColor( image, im_gray, CV_BGR2GRAY );
    cv::Sobel( im_gray, grad_x, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );
    cv::Sobel( im_gray, grad_y, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_y, abs_grad_y );
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges[i] );
//    cv::threshold(edges[i],edges[i], param.grad_threshold, 255, cv::THRESH_BINARY);
//    cv::imshow("edges",edges[i]);
//    cv::waitKey(0);
  }

  // sample data
  sampleCorrespondences(map, lin_images);

  colour_scales_data.assign(3*rgb_correspondences.size(), 1.);

  for (unsigned i=0; i<map.size(); i++)
    optimizeColourScales(i);

  // apply colour scaling and set to map frames
  std::vector< cv::Mat > im_split;

  for (unsigned i=0; i<lin_images.size(); i++)
  {
    cv::split(lin_images[i], im_split);

    for (unsigned j=0; j<3; j++)
      im_split[j] *= colour_scales_data[i*3+j];

    cv::merge(im_split, lin_images[i]);

    rr.directMap(lin_images[i], image);
    v4r::TSFData::setImage(image, map[i]->sf_cloud);

    cout<<"i="<<i<<": "<<colour_scales_data[i*3]<<" "<<colour_scales_data[i*3+1]<<" "<<colour_scales_data[i*3+2]<<" "<<endl;
  }
}


/**
 * setTargetCameraParameter
 */
void OptimizeBundleColours::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows; i++)
      dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }
}

/**
 * @brief OptimizeBundleColours::setParameter
 * @param p
 */
void OptimizeBundleColours::setParameter(const Parameter &p)
{
  param = p;
}

}













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


#include "v4r/keypoints/PlaneEstimationRANSAC.h"

//A makro to get rid of the unused warning
#ifndef UNUSED
#define UNUSED(expr) do { (void)(expr); } while (0)
#endif

namespace v4r
{

using namespace std;


/********************** PlaneEstimationRANSAC ************************
 * Constructor/Destructor
 */
PlaneEstimationRANSAC::PlaneEstimationRANSAC(const Parameter &p)
 : param(p)
{
}

PlaneEstimationRANSAC::~PlaneEstimationRANSAC()
{
}




/************************** PRIVATE ************************/



/**
 * GetRandIdx
 */
void PlaneEstimationRANSAC::getRandIdx(int size, int num, std::vector<int> &idx)
{
  int temp;
  idx.clear();
  for (int i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * GetDistances
 */
void PlaneEstimationRANSAC::getDistances(const std::vector<Eigen::Vector3f> &pts, const Eigen::Vector3f &pt, const Eigen::Vector3f &n, std::vector<float> &dists)
{
  dists.resize(pts.size());

  for (unsigned i = 0; i<pts.size(); i++)
  {
    dists[i] = fabs(normalPointDist(pt, n, pts[i])); 
  }
}

/**
 * countInliers
 */
unsigned PlaneEstimationRANSAC::countInliers(std::vector<float> &dists)
{
  unsigned cnt=0;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < param.inl_dist)
      cnt++; 
  }

  return cnt;
}

/**
 * GetInliers
 */
void PlaneEstimationRANSAC::getInliers(std::vector<float> &dists, std::vector<int> &inliers)
{
  inliers.clear();

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < param.inl_dist)
      inliers.push_back(i);
  }
}

/**
 * Robust estimation of object poses
 */
void PlaneEstimationRANSAC::ransac(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &pt, Eigen::Vector3f &n, std::vector<int> &inliers)
{
  // int ransac
  int k=0;
  float sig=3., sv_sig=0.;
  float eps = sig/(float)pts.size();
  std::vector<int> idx_keys;
  Eigen::Vector3f tmp_n;
  std::vector<float> dists(pts.size());

  //ransac pose
  while (pow(1. - pow(eps,3), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {
    getRandIdx(pts.size(), 3, idx_keys);

    const Eigen::Vector3f &tmp_pt = pts[idx_keys[0]];
    explicitToNormal(tmp_pt, pts[idx_keys[1]], pts[idx_keys[2]], tmp_n);    
  
    getDistances(pts, tmp_pt, tmp_n, dists);
    sig = countInliers(dists);

    if (sig > sv_sig)
    {
      sv_sig = sig;
      pt = tmp_pt;
      n = tmp_n; 
      eps = sv_sig / (float)pts.size(); 
    }

    k++;
  }

  inliers.clear();
  if (sv_sig>3.1)
  {
    getDistances(pts, pt, n, dists);
    getInliers(dists, inliers);

    estimatePlaneLS(pts, inliers, pt, n);

    getDistances(pts, pt, n, dists);
    getInliers(dists, inliers);
  }

  //cout<<"Number of ransac trials: "<<k<<", inl="<<sv_sig<<"("<<inliers.size()<<")/"<<pts.size()<<endl;
}

/**
 * computeCovarianceMatrix
 */
void PlaneEstimationRANSAC::computeCovarianceMatrix (const std::vector<Eigen::Vector3f> &pts, const std::vector<int> &indices, const Eigen::Vector3f &mean, Eigen::Matrix3f &cov)
{
  Eigen::Vector3f pt;
  cov.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    pt = pts[indices[i]] - mean;

    cov(1,1) += pt[1] * pt[1];
    cov(1,2) += pt[1] * pt[2];

    cov(2,2) += pt[2] * pt[2];

    pt *= pt[0];
    cov(0,0) += pt[0];
    cov(0,1) += pt[1];
    cov(0,2) += pt[2];
  }

  cov(1,0) = cov(0,1);
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2);
}





/************************** PUBLIC *************************/

/**
 * estimatePlaneLS
 * least squares estimation of a plan using points defined by indices
 */
void PlaneEstimationRANSAC::estimatePlaneLS(const std::vector<Eigen::Vector3f> &pts, const std::vector<int> &indices, Eigen::Vector3f &pt, Eigen::Vector3f &n)
{
  UNUSED(pt);
  Eigen::Vector3f mean;
  EIGEN_ALIGN16 Eigen::Matrix3f cov;
  

  mean.setZero();

  for (unsigned j=0; j<indices.size(); j++)
    mean += pts[indices[j]];

  mean /= (float)indices.size();

  computeCovarianceMatrix(pts, indices, mean, cov);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> sv(cov);
  Eigen::Matrix3f eigen_vectors = sv.eigenvectors();

  n[0] = eigen_vectors(0,0);
  n[1] = eigen_vectors(1,0);
  n[2] = eigen_vectors(2,0);
}

/**
 * ransacPlane
 * robust plane estimation using RANSAC
 * and a final least squares estimation
 */
void PlaneEstimationRANSAC::ransacPlane(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &pt, Eigen::Vector3f &n, std::vector<int> &inliers)
{
  if (pts.size()<3)
    throw std::runtime_error("[PlaneEstimationRANSAC::ransacPlane] Invalide points!");

  ransac(pts, pt, n, inliers);
}



} //-- THE END --







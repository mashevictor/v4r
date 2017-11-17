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
 * @file   supervoxel_segmentation.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#pragma once

#include <math.h>
#include <stdlib.h>
#include <time.h>

#include <ctime>
#include <iostream>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkPolyLine.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <v4r/core/macros.h>

#define CIE94_KL 1
#define CIE94_K1 0.045
#define CIE94_K2 0.015

using namespace boost::posix_time;

namespace v4r {
template <typename PointInT>
class SupervoxelSegmentation {
  typedef pcl::PointXYZL PointOutT;

 private:
  int mImageWidth;
  int mImageHeight;
  bool mUseCIE94;

  std::map<uint32_t, typename pcl::Supervoxel<PointInT>::Ptr> mSVClusters;
  pcl::PointCloud<pcl::PointXYZL>::Ptr mSVLabeledCloud;
  pcl::PointCloud<pcl::PointXYZL>::Ptr mSVLabeledVoxelCloud;
  typename pcl::PointCloud<PointInT>::Ptr mSVVoxelCentroidCloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mSVColoredCloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mSVColoredVoxelCloud;
  std::multimap<uint32_t, uint32_t> mSVAdjacency;

  static bool sortDistances(std::vector<double> i, std::vector<double> j);

  Eigen::Matrix3f mColorTransformation;

  // mapping of sv labels to cluster numbers
  std::map<unsigned int, unsigned int> mSVToCluster;
  std::vector<std::vector<int>> mClusterToSv;
  int mMergedClusters;

  void RunSupervoxelClustering(typename pcl::PointCloud<PointInT>::ConstPtr input,
                               pcl::PointCloud<pcl::Normal>::ConstPtr normals = NULL);

  int MergeSupervoxels();

  double CalculateDistance(typename pcl::Supervoxel<PointInT>::Ptr svA, typename pcl::Supervoxel<PointInT>::Ptr svB);

  bool Merge2Clusters(typename pcl::Supervoxel<PointInT>::Ptr svA, typename pcl::Supervoxel<PointInT>::Ptr svB);

  void CalculatePoint2PlaneDistances(
      std::vector<Eigen::Vector3f> &centroids, std::vector<Eigen::Vector3f> &normals,
      std::vector<std::vector<std::pair<double, int>>> &sortedPointPlaneDistances,
      std::vector<std::vector<std::pair<double, int>>> &sortedInversePointPlaneDistances);

  void CalculatePairwiseAngleDistances(std::vector<Eigen::Vector3f> &normalVectors, std::vector<double> &verticalAngles,
                                       std::vector<std::vector<std::pair<double, int>>> &sortedVerAngleDifferences,
                                       std::vector<std::vector<std::pair<double, int>>> &sortedHorAngleDifferences);

  void ChangeClusterLabel(unsigned int oldLbl, unsigned int newLbl);

  void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA &supervoxel_center,
                                        pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                        std::string supervoxel_name, pcl::visualization::PCLVisualizer *visualizer,
                                        int &viewport);

  static bool PairwiseComparator(const std::pair<double, int> &l, const std::pair<double, int> r);

  inline double CalculateCIE94Distance(Eigen::Vector3f &labA, Eigen::Vector3f &labB) {
    double a1 = labA[1];
    double a2 = labB[1];
    double b1 = labA[2];
    double b2 = labB[2];

    double dL = labA[0] - labB[0];
    double da = a1 - a2;
    double db = b1 - b2;

    double c1 = sqrt(a1 * a1 + b1 * b1);
    double c2 = sqrt(a2 * a2 + b2 * b2);
    double dC = c1 - c2;
    double dH = sqrt(da * da + db * db - dC * dC);

    double sc = 1 + CIE94_K1 * c1;
    double sh = 1 + CIE94_K2 * c1;

    double termC = dC / sc;
    double termH = dH / sh;

    return sqrt(dL * dL + termC * termC + termH * termH);
  }

  inline Eigen::Vector3f RGB2Lab(Eigen::Vector3f rgb) {
    double R, G, B, r, g, b;
    double X, Y, Z, xr, yr, zr;
    double fx, fy, fz;
    Eigen::Vector3f lab;

    double epsilon = 0.008856;  // actual CIE standard
    double kappa = 903.3;       // actual CIE standard

    const double inv_Xr = 1. / 0.950456;  // reference white
    const double inv_Zr = 1. / 1.088754;  // reference white
    const double inv_12 = 1. / 12.92;
    const double inv_1 = 1. / 1.055;
    const double inv_3 = 1. / 3.0;
    const double inv_116 = 1. / 116.0;

    R = rgb[0];
    G = rgb[1];
    B = rgb[2];

    if (R <= 0.04045)
      r = R * inv_12;
    else
      r = pow((R + 0.055) * inv_1, 2.4);
    if (G <= 0.04045)
      g = G * inv_12;
    else
      g = pow((G + 0.055) * inv_1, 2.4);
    if (B <= 0.04045)
      b = B * inv_12;
    else
      b = pow((B + 0.055) * inv_1, 2.4);

    X = r * 0.4124564 + g * 0.3575761 + b * 0.1804375;
    Y = r * 0.2126729 + g * 0.7151522 + b * 0.0721750;
    Z = r * 0.0193339 + g * 0.1191920 + b * 0.9503041;

    xr = X * inv_Xr;
    yr = Y;  //*inv_Yr;
    zr = Z * inv_Zr;

    if (xr > epsilon)
      fx = pow(xr, inv_3);
    else
      fx = (kappa * xr + 16.0) * inv_116;
    if (yr > epsilon)
      fy = pow(yr, inv_3);
    else
      fy = (kappa * yr + 16.0) * inv_116;
    if (zr > epsilon)
      fz = pow(zr, inv_3);
    else
      fz = (kappa * zr + 16.0) * inv_116;

    lab[0] = 116.0 * fy - 16.0;
    lab[1] = 500.0 * (fx - fy);
    lab[2] = 200.0 * (fy - fz);
    return lab;
  }

 public:
  // supervoxel settings
  float mSVVoxelResolution;
  float mSVSeedResolution;
  float mSVColorImportance;
  float mSVSpatialImportance;
  float mSVNormalImportance;

  // cluster merging settings
  float mMergeThreshold;
  float mMergeColorWeight;
  float mMergeNormalWeight;
  float mMergeCurvatureWeight;
  float mMergePtPlWeight;

  // set to true if input cloud is single frame
  bool mUseSingleCameraTransform;

  SupervoxelSegmentation();

  SupervoxelSegmentation(float sv_voxelResolution, float sv_seedResolution, float sv_colorWeight, float sv_normalWeight,
                         float sv_spatialWeight, float merge_threshold, float merge_colorWeight,
                         float merge_normalWeight, float merge_curvatureWeight, float merge_pointPlaneWeight,
                         bool useCIE94distance, bool useSingleCameraTransform);

  int RunSegmentation(typename pcl::PointCloud<PointInT>::ConstPtr input,
                      pcl::PointCloud<pcl::Normal>::ConstPtr normals = nullptr);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredClusterPointCloud();

  pcl::PointCloud<pcl::PointXYZL>::Ptr getSVlabeledCloud();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getSVColoredCloud();

  pcl::PointCloud<pcl::PointXYZL>::Ptr GetClusterIDPointcloud();

  void GetVoxelizedResults(typename pcl::PointCloud<PointInT>::Ptr voxels, pcl::PointCloud<pcl::Normal>::Ptr normals,
                           pcl::PointCloud<pcl::PointXYZL>::Ptr clusterIDs);

  void GetClusterIDArray(cv::Mat &clusterIDs);

  void GetClusterAdjacency(std::vector<std::vector<int>> &adjacencies);

  void GetEuclideanClusterDistance(std::vector<std::vector<std::pair<double, int>>> &sortedEuclideanDistances);

  void CalculateFeatures(float height, float roll, float pitch, std::vector<std::vector<double>> &features,
                         std::vector<std::vector<std::pair<double, int>>> &sortedEuclideanDistances,
                         std::vector<std::vector<std::pair<double, int>>> &sortedPointPlaneDistances,
                         std::vector<std::vector<std::pair<double, int>>> &sortedInversePointPlaneDistances,
                         std::vector<std::vector<std::pair<double, int>>> &sortedVerAngleDifferences,
                         std::vector<std::vector<std::pair<double, int>>> &sortedHorAngleDifferences);

  void CalculateFeatures2(float height, std::vector<std::vector<double>> &features,
                          std::vector<std::vector<std::pair<double, int>>> &sortedEuclideanDistances,
                          std::vector<std::vector<std::pair<double, int>>> &sortedPointPlaneDistances,
                          std::vector<std::vector<std::pair<double, int>>> &sortedInversePointPlaneDistances,
                          std::vector<std::vector<std::pair<double, int>>> &sortedVerAngleDifferences,
                          std::vector<std::vector<std::pair<double, int>>> &sortedHorAngleDifferences);

  void GetLabels(pcl::PointCloud<pcl::PointXYZL>::Ptr labelCloud, cv::Mat clusterIDArray,
                 std::vector<int> &clusterLabels);

  void GetLabelsForClusters(pcl::PointCloud<pcl::PointXYZL>::Ptr labelCloud, std::vector<int> &clusterLabels);

  void drawSupervoxelAdjacency(pcl::visualization::PCLVisualizer *visualizer, int &viewport);
};
}

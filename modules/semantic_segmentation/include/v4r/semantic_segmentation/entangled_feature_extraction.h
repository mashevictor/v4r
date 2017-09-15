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
 * @file   entangled_feature_extraction.h
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */


#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <math.h>
#include <stdlib.h>

#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <v4r/core/macros.h>
#include <v4r/semantic_segmentation/entangled_data.h>

#define CIE94_KL 1
#define CIE94_K1 0.045
#define CIE94_K2 0.015

namespace v4r
{

    class V4R_EXPORTS EntangledForestFeatureExtraction
    {
    private:
        std::vector< std::vector<double> > mUnaries;
        std::vector<std::vector<std::pair<double, int> > > mPairwisePtPl;
        std::vector<std::vector<std::pair<double, int> > > mPairwiseIPtPl;
        std::vector<std::vector<std::pair<double, int> > > mPairwiseVAngle;
        std::vector<std::vector<std::pair<double, int> > > mPairwiseHAngle;
        std::vector<std::vector<std::pair<double, int> > > mPairwiseEuclid;

        Eigen::Matrix4f mTransformationMatrix;
        double mCameraHeight;

        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr mInputCloud;
        pcl::PointCloud<pcl::Normal>::ConstPtr mNormalCloud;
        pcl::PointCloud<pcl::PointXYZL>::ConstPtr mLabelCloud;

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > mSegments;
        std::vector<pcl::PointCloud<pcl::Normal>::Ptr > mSegmentNormals;

        int mNrOfSegments;

        inline double CalculateCIE94Distance(Eigen::Vector3f& labA, Eigen::Vector3f& labB)
        {
            double a1 = labA[1];
            double a2 = labB[1];
            double b1 = labA[2];
            double b2 = labB[2];

            double dL = labA[0]-labB[0];
            double da = a1-a2;
            double db = b1-b2;

            double c1 = sqrt(a1*a1 + b1*b1);
            double c2 = sqrt(a2*a2 + b2*b2);
            double dC = c1-c2;
            double dH = sqrt(da*da+db*db-dC*dC);

            double sc = 1+CIE94_K1*c1;
            double sh = 1+CIE94_K2*c1;

            double termC = dC / sc;
            double termH = dH / sh;

            return sqrt(dL*dL + termC*termC + termH*termH);
        }

        inline Eigen::Vector3f RGB2Lab(Eigen::Vector3f rgb)
        {
            double R, G, B, r, g, b;
            double X, Y, Z, xr, yr, zr;
            double fx, fy, fz;
            Eigen::Vector3f lab;

            double epsilon = 0.008856;  //actual CIE standard
            double kappa   = 903.3;   //actual CIE standard

            const double inv_Xr = 1./0.950456; //reference white
            const double inv_Zr = 1./1.088754; //reference white
            const double inv_12 = 1./12.92;
            const double inv_1 = 1./1.055;
            const double inv_3 = 1./3.0;
            const double inv_116 = 1./116.0;

            R = rgb[0]; G = rgb[1]; B = rgb[2];

            if(R <= 0.04045)  r = R*inv_12;
            else        r = pow((R+0.055)*inv_1,2.4);
            if(G <= 0.04045)  g = G*inv_12;
            else        g = pow((G+0.055)*inv_1,2.4);
            if(B <= 0.04045)  b = B*inv_12;
            else        b = pow((B+0.055)*inv_1,2.4);

            X = r*0.4124564 + g*0.3575761 + b*0.1804375;
            Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
            Z = r*0.0193339 + g*0.1191920 + b*0.9503041;

            xr = X*inv_Xr;
            yr = Y;//*inv_Yr;
            zr = Z*inv_Zr;

            if(xr > epsilon)  fx = pow(xr, inv_3);
            else        fx = (kappa*xr + 16.0)*inv_116;
            if(yr > epsilon)  fy = pow(yr, inv_3);
            else        fy = (kappa*yr + 16.0)*inv_116;
            if(zr > epsilon)  fz = pow(zr, inv_3);
            else        fz = (kappa*zr + 16.0)*inv_116;

            lab[0] = 116.0*fy-16.0;
            lab[1] = 500.0*(fx-fy);
            lab[2] = 200.0*(fy-fz);
            return lab;
        }

        void CalculatePairwiseFeatures(std::vector<Eigen::Vector3f> &centroids, std::vector<Eigen::Vector3f> &normals, std::vector<double> &verticalAngles);
        void CalculatePairwiseFeaturesOctree(std::vector<Eigen::Vector3f> &centroids, std::vector<Eigen::Vector3f> &normals, std::vector<double> &verticalAngles);

        static bool PairwiseComparator(const std::pair<double, int> &l, const std::pair<double, int> r);
    protected:

    public:
        EntangledForestFeatureExtraction();
        void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::Normal>::ConstPtr normals, pcl::PointCloud<pcl::PointXYZL>::ConstPtr labels, int nlabels);
        void setCameraExtrinsics(double height, double pitch, double roll);
        void setCameraExtrinsics(double height, Eigen::Matrix3f& rotationMatrix);
        void extract();
        void extractFromReconstruction(bool adjustHeight = false);
        void getFeatures(std::vector< std::vector<double> >& unaries, std::vector<std::vector<std::pair<double, int> > >& ptpl, std::vector<std::vector<std::pair<double, int> > >& iptpl,
                         std::vector<std::vector<std::pair<double, int> > >& verAngle, std::vector<std::vector<std::pair<double, int> > >& horAngle,
                         std::vector<std::vector<std::pair<double, int> > >& euclid);
        void prepareClassification(EntangledForestData* d);
    };

}

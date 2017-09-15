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

#include <v4r/features/FeatureSelection.h>
#include <opencv2/highgui/highgui.hpp>


namespace v4r
{
using namespace std;

/************************************************************************************
 * Constructor/Destructor
 */
FeatureSelection::FeatureSelection(const Parameter &p)
 : param(p)
{ 
  rnn.dbg = false;
}

FeatureSelection::~FeatureSelection()
{
}


/***************************************************************************************/

/**
 * compute
 */
void FeatureSelection::compute(std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
//ScopeTime t("FeatureSelection::compute");
  rnn.param.dist_thr = param.thr_image_px;
  std::vector<std::vector<int> > pts_clusters, desc_clusters;
  std::vector<cv::KeyPoint> tmp_keys;
  cv::Mat tmp_descs = cv::Mat_<float>(0,descriptors.cols);
  v4r::DataMatrix2Df centers;

  // cluster keypoints
  pts.clear();
  pts.reserve(keys.size(),2);
  
  for (unsigned i=0; i<keys.size(); i++)
    pts.push_back(&keys[i].pt.x, 2);

  rnn.cluster(pts);
  rnn.getClusters(pts_clusters);

  // cluster descriptors
  rnn.param.dist_thr = param.thr_desc;

//unsigned cnt=0;
//cv::Mat tmp;
//dbg.copyTo(tmp);
  for (unsigned i=0; i<pts_clusters.size(); i++)
  {
    if (pts_clusters[i].size() == 0) continue;

    if (pts_clusters[i].size() == 1)
    {
      tmp_descs.push_back(descriptors.row(pts_clusters[i][0]));
      tmp_keys.push_back(keys[pts_clusters[i][0]]);
//cv::KeyPoint &key = tmp_keys.back();
//cv::circle( tmp, key.pt, key.size/2, cv::Scalar(255), 1, CV_AA );
      continue;
    }

    descs.clear();
    descs.reserve(pts_clusters[i].size(), descriptors.cols);

    for (unsigned j=0; j<pts_clusters[i].size(); j++)
      descs.push_back(&descriptors.at<float>(pts_clusters[i][j],0), descriptors.cols);

    rnn.cluster(descs);
    rnn.getClusters(desc_clusters);
    rnn.getCenters(centers);

    for (unsigned j=0; j<desc_clusters.size(); j++)
    {
      unsigned size = desc_clusters[j].size();
      float inv_size = 1./(float)size;
      tmp_descs.push_back(cv::Mat_<float>(1,descriptors.cols,&centers(j,0)));
      tmp_keys.push_back(cv::KeyPoint(0,0,0,0,0,0,-1));
      cv::KeyPoint &mkey = tmp_keys.back();

      for (unsigned k=0; k<size; k++)
      {
        cv::KeyPoint &key = keys[pts_clusters[i][desc_clusters[j][k]]];
        mkey.pt += key.pt;
        mkey.size += key.size;
        mkey.angle += key.angle;
        mkey.response += key.response;
        mkey.octave += key.octave;
//cv::circle( tmp, key.pt, key.size/2, cv::Scalar(0), 1, CV_AA );
      }

      mkey.pt *= inv_size;
      mkey.size *= inv_size;
      mkey.angle *= inv_size;
      mkey.response *= inv_size;
      mkey.octave *= inv_size;
//cv::KeyPoint &key = tmp_keys.back();
//cv::circle( tmp, key.pt, key.size/2, cv::Scalar(255), 1, CV_AA );
    }

//    cout<<"desc_clusters.size()="<<desc_clusters.size();
//for (unsigned j=0; j<desc_clusters.size(); j++)
//{
//  cv::Mat tmp;
//  dbg.copyTo(tmp);

//  for (unsigned k=0; k<desc_clusters[j].size(); k++)
//  {
//    cv::KeyPoint &key = keys[pts_clusters[i][desc_clusters[j][k]]];
//    cv::circle( tmp, key.pt, key.size/2, cv::Scalar(255), 1, CV_AA );
//  }
//if (desc_clusters[j].size()>=2) {
//  char filename[PATH_MAX];
//  snprintf(filename,PATH_MAX,"log/dbg_%04d.png",cnt);
//  cv::imwrite(filename,tmp);
//  cnt++;
//  cout<<desc_clusters[j].size()<<" ";
//  cout<<" | ";
//}
//}
//cout<<endl;

//cout<<"--"<<endl;
  }

//cv::imwrite("log/dbg.png",tmp);


  tmp_descs.copyTo(descriptors);
  keys = tmp_keys;

  // mark near by points
  /*pts.clear();
  pts.reserve(keys.size(),2);

  for (unsigned i=0; i<keys.size(); i++)
    pts.push_back(&keys[i].pt.x, 2);

  rnn.cluster(pts);
  rnn.getClusters(pts_clusters);

  for (unsigned i=0; i<pts_clusters.size(); i++)
  {
    for (unsigned j=1; j<pts_clusters[i].size(); j++)
      keys[pts_clusters[i][j]].response = -1;
  }*/
}
}

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
 * @file IMKObjectVotesClustering.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */


#include <v4r/recognition/IMKObjectVotesClustering.h>
#include <v4r/common/impl/Vector.hpp>
#include <v4r/keypoints/impl/triple.hpp>

#include "opencv2/highgui/highgui.hpp"
//#include "v4r/KeypointTools/ScopeTime.hpp"

//#define DEBUG_VOTE_CL


namespace v4r
{

using namespace std;

const float IMKObjectVotesClustering::two_pi = 2.*M_PI;



inline bool cmpCluster(const boost::shared_ptr<v4r::triple<unsigned, double, std::vector< cv::DMatch > > > &a, const boost::shared_ptr<v4r::triple<unsigned, double, std::vector< cv::DMatch > > > &b)
{
  return (a->second>b->second);
}





/************************************************************************************
 * Constructor/Destructor
 */
IMKObjectVotesClustering::IMKObjectVotesClustering(const Parameter &p)
 : param(p)
{
}

IMKObjectVotesClustering::~IMKObjectVotesClustering()
{
}

/**
 * @brief IMKObjectVotesClustering::createVotes
 * @param id
 * @param views
 * @param keys
 * @param matches
 * @param votes
 * @param voting_matches
 */
void IMKObjectVotesClustering::createVotes(unsigned id, const std::vector<IMKView> &views, const std::vector<cv::KeyPoint> &keys, const std::vector< std::vector< cv::DMatch > > &matches, DataMatrix2Df &_votes, std::vector< cv::DMatch > &_voting_matches, std::vector<double> &_weights)
{
  float scale, delta_angle;
  cv::Point2f pt, pt_scaled;

  _votes.resize(0,2);
  _voting_matches.clear();
  _weights.clear();


#ifdef DEBUG_VOTE_CL
  cv::Mat im_tmp;
  if (!dbg.empty())
    dbg.copyTo(im_tmp);
#endif

  for (unsigned i=0; i<matches.size(); i++)
  {
    const std::vector< cv::DMatch > &ms = matches[i];
    for (unsigned j=0; j<ms.size(); j++)
    {
      const cv::DMatch &m = ms[j];
      if (views[m.imgIdx].object_id == id)
      {
        const cv::KeyPoint &query_key = keys[m.queryIdx];
        const cv::KeyPoint &train_key = views[m.imgIdx].keys[m.trainIdx];

        scale = query_key.size/train_key.size;
        delta_angle = diffAngle_0_2pi(query_key.angle*M_PI/180., train_key.angle*M_PI/180.);

        pt_scaled = scale*train_key.pt;
        rotate2(&pt_scaled.x, delta_angle, &pt.x);
        pt = pt+query_key.pt;

//    pt = train_key.pt+query_key.pt;

        _weights.push_back(1./(double)ms.size());
        _voting_matches.push_back(m);
        _votes.push_back(&pt.x, 2);

#ifdef DEBUG_VOTE_CL
  if (!dbg.empty())
  {
    cv::line(im_tmp, query_key.pt, pt, CV_RGB(255,255,255), 1);
    cv::circle(im_tmp, pt, 2, CV_RGB(255,0,0),2);
  }
#endif

      }
    }
  }

#ifdef DEBUG_VOTE_CL
  if (!dbg.empty())
  {
    cv::imshow("dbg votes", im_tmp);
//    cv::waitKey(0);
  }
#endif
}



/******************************* PUBLIC ***************************************/

/**
 * @brief IMKObjectVotesClustering::operate
 * @param object_names
 * @param views
 * @param keys
 * @param matches
 * @param clusters DMatch::distance .. 1./number of occurances
 */
void IMKObjectVotesClustering::operate(const std::vector<std::string> &object_names, const std::vector<IMKView> &views, const std::vector<cv::KeyPoint> &keys, const std::vector< std::vector< cv::DMatch > > &matches, std::vector< boost::shared_ptr< v4r::triple<unsigned, double, std::vector< cv::DMatch > > > > &clusters)
{
  rnn.param.dist_thr = param.cluster_dist;
  rnn.dbg = false;
  clusters.clear();

  for (unsigned i=0; i<object_names.size(); i++)
  {
    createVotes(i, views, keys, matches, votes, voting_matches, weights);

    rnn.cluster(votes);
    rnn.getClusters(indices);

    for (unsigned j=0; j<indices.size(); j++)
    {
      const std::vector<int> &inds = indices[j];
      clusters.push_back( boost::shared_ptr< v4r::triple<unsigned, double, std::vector< cv::DMatch > > >(new v4r::triple<unsigned, double, std::vector< cv::DMatch > >()) );
      v4r::triple<unsigned, double, std::vector< cv::DMatch > > &cl = *clusters.back();
      cl.first = i;
      cl.second = 0.;
      for (unsigned k=0; k<inds.size(); k++)
      {
        cl.third.push_back(voting_matches[inds[k]]);
        cl.third.back().distance = weights[inds[k]];
        cl.second += weights[inds[k]];
      }
//      if (cl.second>10) cout<<inds.size()<<"("<<cl.second<<") ";
    }
  }
//  cout<<endl;

  // sort clusters
  std::sort(clusters.begin(), clusters.end(), cmpCluster);


#ifdef DEBUG_VOTE_CL
  if (!dbg.empty())
  {
    cv::Mat im_tmp;
    float scale, delta_angle;
    cv::Point2f pt, pt_scaled;
    dbg.copyTo(im_tmp);

    for (unsigned i=0; i<clusters.size() && i<50; i++)
    {
//      dbg.copyTo(im_tmp);
      const v4r::triple<unsigned, double, std::vector< cv::DMatch > > &cl = *clusters[i];
      cv::Vec3b col(rand()%255,rand()%255,rand()%255);

      cout<<i<<": obj="<<cl.first<<", weight="<<cl.second<<endl;
      if (cl.second<4) break;

      for (unsigned j=0; j<cl.third.size(); j++)
      {
        const cv::DMatch &m = cl.third[j];
        const cv::KeyPoint &query_key = keys[m.queryIdx];
        const cv::KeyPoint &train_key = views[m.imgIdx].keys[m.trainIdx];

        scale = query_key.size/train_key.size;
        delta_angle = diffAngle_0_2pi(query_key.angle*M_PI/180., train_key.angle*M_PI/180.);

        pt_scaled = scale*train_key.pt;
        rotate2(&pt_scaled.x, delta_angle, &pt.x);
        pt = pt+query_key.pt;

        cv::line(im_tmp, pt, query_key.pt, CV_RGB(255,255,255), 1);
        cv::circle(im_tmp, pt, 2, CV_RGB(col[0],col[1],col[2]),2);
      }

      cv::imshow("dbg votes", im_tmp);
//      cv::waitKey(0);
    }

    cout<<"-- end --"<<endl;
    cv::waitKey(0);
  }
#endif
}




}













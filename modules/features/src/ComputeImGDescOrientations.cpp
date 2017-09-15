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

#include <v4r/features/ComputeImGDescOrientations.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace v4r
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
ComputeImGDescOrientations::ComputeImGDescOrientations(const Parameter &p)
 : param(p)
{ 
  h_win = param.win_size/2;
}

ComputeImGDescOrientations::~ComputeImGDescOrientations()
{
}


/***************************************************************************************/

/**
 * compute descriptors
 */
void ComputeImGDescOrientations::compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::Point2f> &pts, std::vector<cv::KeyPoint> &keys)
{
  ImGDescOrientation gradOri(param.goParam);

  keys.resize(pts.size());
  int wsize2 = param.win_size-2;

  #pragma omp parallel for private(gradOri)
  for (unsigned i=0; i<pts.size(); i++)
  {
    const cv::Point2f &pt = pts[i];
    cv::KeyPoint &key = keys[i];
    key = cv::KeyPoint(pt, wsize2, 0);

    if (pt.x-h_win>=0 && pt.y-h_win>=0 && (pt.x+h_win<image.cols && pt.y+h_win<image.rows))
    {
      gradOri.compute(cv::Mat(image, cv::Rect(pt.x-h_win,pt.y-h_win, param.win_size,param.win_size)), key.angle);
    }
  }
 
}

/**
 * compute descriptors
 */
void ComputeImGDescOrientations::compute(const cv::Mat_<unsigned char> &image, std::vector<cv::KeyPoint> &keys)
{
  ImGDescOrientation gradOri(param.goParam);

  cv::Mat_<float> M(2,3);
  cv::Mat_<unsigned char> im_desc(param.win_size,param.win_size);
  cv::Size dsize(param.win_size,param.win_size);
 
  float sin_dir = 0;

  #pragma omp parallel for private(gradOri, im_desc, M)
  for (unsigned i=0; i<keys.size(); i++)
  {
    cv::KeyPoint &key = keys[i];
    float cos_dir = (param.win_size-2)/key.size;   // just scaling

    M = cv::Mat_<float>(2,3);
    M(0,0)= cos_dir;
    M(0,1)= -sin_dir;
    M(0,2)= h_win - cos_dir*key.pt.x + sin_dir*key.pt.y;
    M(1,0)= sin_dir;
    M(1,1)= cos_dir;
    M(1,2)= h_win - sin_dir*key.pt.x - cos_dir*key.pt.y; 
    
    cv::warpAffine(image, im_desc, M, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    gradOri.compute(im_desc, key.angle);
  }
 
}

/**
 * compute descriptors
 */
/*void ComputeImGDescOrientations::compute(const cv::Mat_<unsigned char> &image, std::vector<AffKeypoint> &keys)
{
  ImGDescOrientation gradOri(param.goParam);

  cv::Mat_<float> M(2,3);
  cv::Mat_<unsigned char> im_desc(param.win_size,param.win_size);
  cv::Size dsize(param.win_size,param.win_size);

  float sin_dir = 0.;
  
//cv::Mat dbg;
//image.copyTo(dbg);


  #pragma omp parallel for private(gradOri, im_desc, M)
  for (unsigned i=0; i<keys.size(); i++)
  {
    AffKeypoint &key = keys[i];

    float s = 1./(key.mi11*key.mi22-key.mi12*key.mi21);
    float cos_dir = .5 * s * (param.win_size-2)/key.size;    // just scaling

    M = cv::Mat_<float>(2,3);
    M(0,0)= cos_dir*key.mi22 - sin_dir*key.mi21;
    M(0,1)= cos_dir*key.mi12 + sin_dir*key.mi11;
    M(0,2)= h_win - key.pt.x*M(0,0) - key.pt.y*M(0,1);
    M(1,0)= -sin_dir*key.mi22 + cos_dir*key.mi21;
    M(1,1)= sin_dir*key.mi12 + cos_dir*key.mi11;
    M(1,2)= h_win - key.pt.x*M(1,0) - key.pt.y*M(1,1);
    
    cv::warpAffine(image, im_desc, M, dsize, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    gradOri.compute(im_desc, key.angle);

//cout<<"key.angle"<<key.angle<<endl;
//AffKeypoint::Draw(dbg,key,cv::Scalar(255));
//cv::imshow("image", dbg);
//cv::imshow("dbg", im_desc);
//cv::waitKey(0);
  }
}*/
}


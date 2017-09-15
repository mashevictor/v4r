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



#include "v4r/attention_segmentation/drawUtils.h"

namespace v4r
{

void drawSegmentationMask(cv::Mat &image, cv::Mat mask, cv::Scalar color, int line_width)
{
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  if(contours.size())
  {
    int contour_idx = 0;
    // extract only maximal contour
    for(unsigned int j = 1; j < contours.size(); ++j)
    {
      if(contours.at(j).size() > contours.at(contour_idx).size())
      {
        contour_idx = j;
      }
    }
    cv::drawContours(image,contours,contour_idx,color,line_width);
  }
}
  
void drawSegmentationMasks(cv::Mat &image, std::vector<cv::Mat> &masks, int line_width)
{
  for(unsigned int i = 0; i < masks.size(); ++i)
  {
    // draw contour
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;

    drawSegmentationMask(image,masks.at(i),cv::Scalar(r,g,b),line_width);
  }
}

void drawSegmentationResults(cv::Mat &image, cv::Point p1, cv::Mat &masks, bool drawAttentionPoints,
                             bool drawSegmentationResults, int num)
{
  cv::Scalar color;
  // draw contour
  int r = rand() % 256;
  int g = rand() % 256;
  int b = rand() % 256;
    
  color = cv::Scalar(r,g,b);
  
  if(drawSegmentationResults)
  {
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(masks,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    cv::drawContours(image,contours,0,color,3);
  }
  
  if(drawAttentionPoints)
  {
    // put text
    char text[4];
    sprintf(text,"%d",num);
    cv::putText(image,text,cv::Point(p1.x,p1.y-5),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,0),2);//colors.at(j),2);
    // draw attention point
    cv::circle(image,p1,5,color,-1);
  }
}

void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<std::vector<cv::Point> > &contours, bool drawAttentionPoints,
                             bool drawSegmentationResults, bool drawLines, unsigned int num)
{
  cv::Point p;
  unsigned int N = 0;
  if(drawAttentionPoints)
    N = attentionPoints.size();
  if(drawSegmentationResults)
    N = (N > contours.size() ? N : contours.size());

  std::vector<cv::Scalar> colors;
  
  for(unsigned int j = 0; j < N; ++j)
  {
    // draw contour
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;

    if(j == 0)
    {
      r = 0;
      g = 255;
      b = 0;
    }
    if(j == 1)
    {
      r = 255;
      g = 0;
      b = 0;
    }
    if(j == 2)
    {
      r = 0;
      g = 0;
      b = 255;
    }
    
    colors.push_back(cv::Scalar(r,g,b));
    
  }
  
  if(num >= 0)
    N = (N > num ? num : N);

  for(unsigned int j = 0; j < N; ++j)
  {
    if(drawSegmentationResults && (contours.size() > j))
    {
      cv::drawContours(image,contours,j,colors.at(j),3);
    }
  }
  
  for(unsigned int j = 0; j < N; ++j)
  {
    if(attentionPoints.size() > j)
    {
      cv::Point p1 = attentionPoints.at(j);
      if(drawAttentionPoints)
      {
        // put text
        char text[4];
        sprintf(text,"%d",j);
        cv::putText(image,text,cv::Point(p1.x,p1.y-5),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(0,0,0),2);//colors.at(j),2);
        if (j > 0)
        {
          if(drawLines)
	    cv::line(image,p,p1,cv::Scalar(255,255,255),2);
        }

        // draw attention point
        cv::circle(image,p1,5,colors.at(j),-1);
      }

      p = p1;
    }
  }
}

void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<cv::Mat> &binMasks, std::vector<std::vector<cv::Point> > &contours,
                             bool drawAttentionPoints, bool drawSegmentationResults)
{
  cv::Point p;
  unsigned int N = (attentionPoints.size() > binMasks.size() ?
                    attentionPoints.size() : attentionPoints.size());
  for(unsigned int j = 0; j < N; ++j)
  {
    // draw contour
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;

    if(drawSegmentationResults && (binMasks.size() > j))
    {
      cv::drawContours(image,contours,j,cv::Scalar(r,g,b),2);
    }

    if(attentionPoints.size() > j)
    {
      cv::Point p1 = attentionPoints.at(j);
      if(drawAttentionPoints)
      {
        // put text
        char text[2];
        sprintf(text,"%d",j);
        cv::putText(image,text,cv::Point(p1.x,p1.y-5),cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(r,g,b));
        if (j > 0)
        {
          cv::line(image,p,p1,cv::Scalar(255,255,255),1);
        }

        // draw attention point
        cv::circle(image,p1,5,cv::Scalar(r,g,b),-1);
      }

      p = p1;
    }
  }
}

//revison
void drawAttentionPoints(cv::Mat &image, std::vector<cv::Point> &attentionPoints, unsigned int maxNumber, bool connect_points)
{
  cv::Point p;
  maxNumber = (maxNumber > 0 ? maxNumber : attentionPoints.size());
  maxNumber = (maxNumber > attentionPoints.size() ? attentionPoints.size() : maxNumber);
  for(unsigned int j = 0; j < maxNumber; ++j)
  {
    cv::Point p1 = attentionPoints.at(j);

    // select color
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;

    // put text
    char text[4];
    sprintf(text,"%d",j);
    cv::putText(image,text,cv::Point(p1.x,p1.y-10),cv::FONT_HERSHEY_SIMPLEX,2.0,cv::Scalar(r,g,b),5);
    
    if (connect_points && (j > 0))
    {
      cv::line(image,p,p1,cv::Scalar(255,255,255),1);
    }

    // draw attention point
    cv::circle(image,p1,10,cv::Scalar(r,g,b),-1);

    p = p1;
  }
}
//end revision

void drawPath(cv::Mat &image, std::vector<cv::Point> &path, cv::Mat &mapx, cv::Mat &mapy)
{
  for(unsigned int i = 0; i < path.size(); ++i)
  {
    int phi = path.at(i).y;
    int rho = path.at(i).x;
    int x = mapx.at<float>(phi,rho);
    int y = mapy.at<float>(phi,rho);
    image.at<float>(y,x) = 1;
  }
}

void drawLine(cv::Mat &image, std::vector<cv::Point> points, cv::Scalar color)
{
  for(unsigned int i = 0; i < points.size(); ++i)
  {
    int x = points.at(i).x;
    int y = points.at(i).y;
    
    image.at<uchar>(y,3*x+0) = color(0);
    image.at<uchar>(y,3*x+1) = color(1);
    image.at<uchar>(y,3*x+2) = color(2);
  }
}

} //namespace v4r

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
 * @file color_transforms.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2013
 * @brief
 *
 */

#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <v4r/core/macros.h>
#include <vector>

#include <omp.h>

namespace v4r
{

class V4R_EXPORTS ColorTransform
{
public:
    typedef boost::shared_ptr< ColorTransform > Ptr;

    virtual ~ColorTransform() {}

    virtual
    Eigen::VectorXf
    do_conversion(unsigned char R, unsigned char G, unsigned char B) const = 0;

    virtual void
    do_inverse_conversion(const Eigen::VectorXf &converted_color, unsigned char &R, unsigned char &G, unsigned char &B) const
    {
        (void) converted_color;
        (void) R;
        (void) G;
        (void) B;
        std::cerr << "Inverse conversion is not implemented!" << std::endl;
    }

    virtual size_t getOutputNumColorCompenents() const = 0;

    template<typename PointT>
    V4R_EXPORTS void
    convert(const pcl::PointCloud<PointT> &cloud, Eigen::MatrixXf &converted_color) const
    {
        converted_color = Eigen::MatrixXf (cloud.points.size(), getOutputNumColorCompenents());

#pragma omp parallel for schedule (dynamic)
        for(size_t i=0; i < cloud.points.size(); i++)
        {
            const PointT &p = cloud.points[i];
            unsigned char r = (unsigned char)p.r;
            unsigned char g = (unsigned char)p.g;
            unsigned char b = (unsigned char)p.b;
            converted_color.row(i) = do_conversion( r, g, b);
        }
    }
};


class V4R_EXPORTS RGB2GrayScale : public ColorTransform
{
public:
    typedef boost::shared_ptr< RGB2GrayScale > Ptr;

    size_t getOutputNumColorCompenents() const { return 1; }

    Eigen::VectorXf
    do_conversion(unsigned char R, unsigned char G, unsigned char B) const
    {
        Eigen::VectorXf c(1);
        c(0) = 0.2126f * R/255.f + 0.7152f * G/255.f + 0.0722f * B/255.f;
        return c;
    }
};

}

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


#ifndef NURBS_DATA_H
#define NURBS_DATA_H

#include <vector>
#include <list>
#include <stdio.h>

#undef Success
#include <Eigen/StdVector>

namespace pcl
{
  namespace on_nurbs
  {

    // http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html
    typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > vector_vec2i;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector_vec2d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
    typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vector_vec4d;

    /** \brief Data structure for NURBS surface fitting
     * (FittingSurface, FittingSurfaceTDM, FittingCylinder, GlobalOptimization, GlobalOptimizationTDM) */
    struct NurbsDataSurface
    {
      Eigen::Matrix3d eigenvectors;
      Eigen::Vector3d mean;

      vector_vec3d interior; ///<< input
      std::vector<double> interior_weight; ///<< input
      std::vector<double> interior_error; ///>> output
      vector_vec2d interior_param; ///>> output
      vector_vec3d interior_line_start; ///>> output
      vector_vec3d interior_line_end; ///>> output
      vector_vec3d interior_normals; ///>> output

      vector_vec3d boundary; ///<< input
      std::vector<double> boundary_weight; ///<< input
      std::vector<double> boundary_error; ///>> output
      vector_vec2d boundary_param; ///>> output
      vector_vec3d boundary_line_start; ///>> output
      vector_vec3d boundary_line_end; ///>> output
      vector_vec3d boundary_normals; ///>> output

      vector_vec3d common_boundary_point;
      std::vector<unsigned> common_boundary_idx;
      vector_vec2d common_boundary_param;

      std::vector<unsigned> common_idx;
      vector_vec2d common_param1;
      vector_vec2d common_param2;

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_weight.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      /** \brief Clear all boundary data */
      inline void
      clear_boundary ()
      {
        boundary.clear ();
        boundary_weight.clear ();
        boundary_error.clear ();
        boundary_param.clear ();
        boundary_line_start.clear ();
        boundary_line_end.clear ();
        boundary_normals.clear ();
      }

      inline void
      clear_common()
      {
        common_idx.clear();
        common_param1.clear();
        common_param2.clear();
      }

      /** \brief Clear all common data */
      inline void
      clear_common_boundary ()
      {
        common_boundary_point.clear ();
        common_boundary_idx.clear ();
        common_boundary_param.clear ();
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief Data structure for 3D NURBS curve fitting
     * (FittingCurve) */
    struct NurbsDataCurve
    {
      Eigen::Matrix3d eigenvectors;
      Eigen::Vector3d mean;

      vector_vec3d interior; ///<< input
      std::vector<double> interior_error; ///>> output
      std::vector<double> interior_param; ///>> output
      vector_vec3d interior_line_start; ///>> output
      vector_vec3d interior_line_end; ///>> output
      vector_vec3d interior_normals; ///>> output

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief Data structure for 2D NURBS curve fitting
     * (FittingCurve2d, FittingCurve2dTDM, FittingCurve2dSDM) */
    struct NurbsDataCurve2d
    {
      Eigen::Matrix2d eigenvectors;
      Eigen::Vector2d mean;

      vector_vec2d interior; ///<< input
      std::vector<double> interior_error; ///>> output
      std::vector<double> interior_param; ///>> output
      vector_vec2d interior_line_start; ///>> output
      vector_vec2d interior_line_end; ///>> output
      std::vector<unsigned> interior_line_flag; ///>> output
      vector_vec2d interior_normals; ///>> output

      std::vector<double> interior_weight;
      std::vector<bool> interior_weight_function;

      vector_vec2d closest_points;
      std::vector<double> closest_points_param;
      std::vector<double> closest_points_error;

      int interior_ncps_prev;
      int closest_ncps_prev;

      vector_vec2d interior_tangents;
      std::vector<double> interior_rho;

      vector_vec2d closest_tangents;
      vector_vec2d closest_normals;
      std::vector<double> closest_rho;

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

#endif


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


#ifndef SEQUENTIAL_FITTER_H
#define SEQUENTIAL_FITTER_H

// remove multiple #defines from xlib and OpenMesh
#undef True
#undef False
#undef None
#undef Status

#include <v4r/core/macros.h>

#include "v4r/attention_segmentation/fitting_surface_pdm.h"
#include "v4r/attention_segmentation/nurbs_data.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//#include "v4r/NurbsConvertion/DataLoading.h"

namespace pcl
{
  namespace on_nurbs
  {
    class V4R_EXPORTS SequentialFitter
    {
      public:
        struct Parameter
        {
          int order;
          int refinement;
          int iterationsQuad;
          int iterationsBoundary;
          int iterationsAdjust;
          int iterationsInterior;
          double forceBoundary;
          double forceBoundaryInside;
          double forceInterior;
          double stiffnessBoundary;
          double stiffnessInterior;
          int resolution;
          Parameter (int order = 3, int refinement = 1, int iterationsQuad = 5, int iterationsBoundary = 5,
                     int iterationsAdjust = 5, int iterationsInterior = 2, double forceBoundary = 200.0,
                     double forceBoundaryInside = 400.0, double forceInterior = 1.0, double stiffnessBoundary = 20.0,
                     double stiffnessInterior = 0.1, int resolution = 16);
        };

      private:
        Parameter m_params;

        NurbsDataSurface m_data;
        ON_NurbsSurface m_nurbs;

        ON_3dPoint m_corners[4];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
        pcl::PointIndices::Ptr m_boundary_indices;
        pcl::PointIndices::Ptr m_interior_indices;

        Eigen::Matrix4d m_intrinsic;
        Eigen::Matrix4d m_extrinsic;

        bool m_have_cloud;
        bool m_have_corners;

        int m_surf_id;

        void
        compute_quadfit ();
        void
        compute_refinement (FittingSurface* fitting);
        void
        compute_boundary (FittingSurface* fitting);
        void
        compute_interior (FittingSurface* fitting);

        Eigen::Vector2d
        project (const Eigen::Vector3d &pt);

        bool
        is_back_facing (const Eigen::Vector3d &v0, 
                        const Eigen::Vector3d &v1, 
                        const Eigen::Vector3d &v2,
                        const Eigen::Vector3d &v3);

      public:
        SequentialFitter (Parameter p = Parameter ());

        inline void
        setParams (const Parameter &p)
        {
          m_params = p;
        }

        /** \brief Set input point cloud **/
        void
        setInputCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

        /** \brief Set boundary points of input point cloud **/
        void
        setBoundary (pcl::PointIndices::Ptr &pcl_cloud_indices);

        /** \brief Set interior points of input point cloud **/
        void
        setInterior (pcl::PointIndices::Ptr &pcl_cloud_indices);

        /** \brief Set corner points of input point cloud **/
        void
        setCorners (pcl::PointIndices::Ptr &corners, bool flip_on_demand = true);

        /** \brief Set camera- and world matrices, for projection and back-face detection
         *  \param[in] intrinsic The camera projection matrix.
         *  \param[in] intrinsic The world matrix.*/
        void
        setProjectionMatrix (const Eigen::Matrix4d &intrinsic, 
                             const Eigen::Matrix4d &extrinsic);

        /** \brief Compute point cloud and fit (multiple) models */
        ON_NurbsSurface
        compute (bool assemble = false);

        /** \brief Compute boundary points and fit (multiple) models
         * (without interior points - minimal curvature surface) */
        ON_NurbsSurface
        compute_boundary (const ON_NurbsSurface &nurbs);

        /** \brief Compute interior points and fit (multiple) models
         *  (without boundary points)*/
        ON_NurbsSurface
        compute_interior (const ON_NurbsSurface &nurbs);

        /** \brief Get resulting NURBS surface. */
        inline ON_NurbsSurface
        getNurbs ()
        {
          return m_nurbs;
        }

        /** \brief Get error of each interior point (L2-norm of point to closest point on surface) and square-error */
        void
        getInteriorError (std::vector<double> &error);

        /** \brief Get error of each boundary point (L2-norm of point to closest point on surface) and square-error */
        void
        getBoundaryError (std::vector<double> &error);

        /** \brief Get parameter of each interior point */
        void
        getInteriorParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params);

        /** \brief Get parameter of each boundary point */
        void
        getBoundaryParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params);

        /** \brief get the normals to the interior points given by setInterior() */
        void
        getInteriorNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normal);

        /** \brief get the normals to the boundary points given by setBoundary() */
        void
        getBoundaryNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals);

        /** \brief Get the closest point on a NURBS from a point pt in parameter space
         *  \param[in] nurbs  The NURBS surface
         *  \param[in] pt     A point in 3D from which the closest point is calculated
         *  \param[out] params The closest point on the NURBS in parameter space
         *  \param[in] maxSteps Maximum iteration steps
         *  \param[in] accuracy Accuracy below which the iterations stop */
        static void
        getClosestPointOnNurbs (ON_NurbsSurface nurbs, 
                                const Eigen::Vector3d &pt, 
                                Eigen::Vector2d& params, 
                                int maxSteps = 100,
                                double accuracy = 1e-4);

        /** \brief Growing algorithm (TODO: under construction) */
        ON_NurbsSurface
        grow (float max_dist = 1.0f, float max_angle = M_PI_4, unsigned min_length = 0, unsigned max_length = 10);

        /** \brief Convert point-cloud */
        static unsigned
        PCL2ON (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, const std::vector<int> &indices, vector_vec3d &cloud);

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}

#endif

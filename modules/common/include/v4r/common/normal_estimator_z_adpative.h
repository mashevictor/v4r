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
 * @file normal_estimator_z_adaptive.h
 * @author Andreas Richtsfeld, Johann Prankl (prankl@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2012
 * @brief
 *
 */

#pragma once

#include <Eigen/Dense>
#include <v4r/core/macros.h>
#include <v4r/common/normal_estimator.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r
{

/**
 * @brief The ZAdaptiveNormalsParameter class represent the parameter for ZAdpativeNormals estimation
 */
class V4R_EXPORTS ZAdaptiveNormalsParameter
{
public:
    float radius_;            ///< euclidean inlier radius
    int kernel_;               ///< kernel radius [px]
    bool adaptive_;            ///< Activate z-adaptive normals calcualation
    float kappa_;              ///< gradient
    float d_;                  ///< constant
    std::vector<int> kernel_radius_;   ///< Kernel radius for each 0.5 meter intervall (e.g. if 8 elements, then 0-4m)
    ZAdaptiveNormalsParameter(
            float radius=0.02,
            int kernel=5,
            bool adaptive=false,
            float kappa=0.005125,
            float d = 0.0,
            std::vector<int> kernel_radius = {3,3,3,3,4,5,6,7}
            )
        : radius_(radius),
          kernel_(kernel),
          adaptive_(adaptive),
          kappa_(kappa),
          d_(d),
          kernel_radius_ (kernel_radius)
    {}


    /**
     * @brief init parameters
     * @param command_line_arguments (according to Boost program options library)
     * @return unused parameters (given parameters that were not used in this initialization call)
     */
    std::vector<std::string>
    init(int argc, char **argv)
    {
        std::vector<std::string> arguments(argv + 1, argv + argc);
        return init(arguments);
    }

    /**
     * @brief init parameters
     * @param command_line_arguments (according to Boost program options library)
     * @return unused parameters (given parameters that were not used in this initialization call)
     */
    std::vector<std::string>
    init(const std::vector<std::string> &command_line_arguments)
    {
        po::options_description desc("Surface Normal Estimator Parameter\n=====================\n");
        desc.add_options()
                ("help,h", "produce help message")
                ("normals_radius", po::value<float>(&radius_)->default_value(radius_), "euclidean inlier radius for normal computation.")
                ("normals_kernel_size", po::value<int>(&kernel_)->default_value(kernel_), "kernel radius in pixel.")
                ("normals_z_adaptive", po::value<bool>(&adaptive_)->default_value(adaptive_), "if true, adapts kernel radius with distance of point to camera.")
                ("normals_kappa", po::value<float>(&kappa_)->default_value(kappa_), "gradient.")
                ("normals_d", po::value<float>(&d_)->default_value(d_), "constant.")
                ;
        po::variables_map vm;
        po::parsed_options parsed = po::command_line_parser(command_line_arguments).options(desc).allow_unregistered().run();
        std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
        po::store(parsed, vm);
        if (vm.count("help")) { std::cout << desc << std::endl; to_pass_further.push_back("-h"); }
        try { po::notify(vm); }
        catch(std::exception& e) {  std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl; }
        return to_pass_further;
    }
};


template<typename PointT>
class V4R_EXPORTS ZAdaptiveNormalsPCL : public NormalEstimator<PointT>
{
public:
    using NormalEstimator<PointT>::input_;
    using NormalEstimator<PointT>::indices_;
    using NormalEstimator<PointT>::normal_;

private:
    ZAdaptiveNormalsParameter param_;

    float sqr_radius;

    void computeCovarianceMatrix ( const std::vector<int> &indices, const Eigen::Vector3f &mean, Eigen::Matrix3f &cov);
    void getIndices(size_t u, size_t v, int kernel, std::vector<int> &indices);
    float computeNormal(std::vector<int> &indices,  Eigen::Matrix3d &eigen_vectors);

    int getIdx(short x, short y) const
    {
        return y*input_->width+x;
    }

public:
    ZAdaptiveNormalsPCL(const ZAdaptiveNormalsParameter &p = ZAdaptiveNormalsParameter())
        : param_(p)
    {
        sqr_radius = p.radius_*p.radius_;
    }

    ~ZAdaptiveNormalsPCL(){}

    pcl::PointCloud<pcl::Normal>::Ptr
    compute ();

    int
    getNormalEstimatorType() const
    {
        return NormalEstimatorType::Z_ADAPTIVE;
    }

    typedef boost::shared_ptr< ZAdaptiveNormalsPCL> Ptr;
    typedef boost::shared_ptr< ZAdaptiveNormalsPCL const> ConstPtr;
};

}


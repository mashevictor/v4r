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
 * @file normal_estimator_pcl.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2016
 * @brief
 *
 */
#pragma once

#include <v4r/common/normal_estimator.h>
#include <v4r/core/macros.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace v4r
{

class V4R_EXPORTS NormalEstimatorPCLParameter
{
public:
    float radius_; ///< smoothings size.
    bool use_omp_; ///< use depth depended smoothing

    NormalEstimatorPCLParameter (
            float radius = 0.02f,
            bool use_omp = true
            )
        :
          radius_ ( radius ),
          use_omp_ ( use_omp )
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
                ("normals_radius", po::value<float>(&radius_)->default_value(radius_), "support radius for computation")
                ("normals_use_omp", po::value<bool>(&use_omp_)->default_value(use_omp_), "if true, uses openmp.")
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
class V4R_EXPORTS NormalEstimatorPCL : public NormalEstimator<PointT>
{
public:
    using NormalEstimator<PointT>::input_;
    using NormalEstimator<PointT>::indices_;
    using NormalEstimator<PointT>::normal_;

private:
    NormalEstimatorPCLParameter param_;

public:
    NormalEstimatorPCL(
            const NormalEstimatorPCLParameter &p = NormalEstimatorPCLParameter()
            )
        : param_(p)
    {
    }

    ~NormalEstimatorPCL(){}

    pcl::PointCloud<pcl::Normal>::Ptr
    compute ();

    int
    getNormalEstimatorType() const
    {
        return NormalEstimatorType::PCL_INTEGRAL_NORMAL;
    }

    typedef boost::shared_ptr< NormalEstimatorPCL> Ptr;
    typedef boost::shared_ptr< NormalEstimatorPCL const> ConstPtr;
};

}


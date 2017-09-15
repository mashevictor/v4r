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
 * @file global_color_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */
#pragma once

#include <v4r/common/color_transforms.h>
//#include <v4r/common/rgb2cielab.h>
#include <v4r/core/macros.h>
#include <v4r/features/global_estimator.h>
#include <v4r/features/types.h>

#include <glog/logging.h>

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/serialization.hpp>

#include <fstream>

namespace po = boost::program_options;

namespace v4r
{

class V4R_EXPORTS GlobalColorEstimatorParameter
{
 public:
    size_t num_bins; ///< number of bins for each color chanel to create color histogram
    float std_dev_multiplier_; ///< multiplication factor of the standard deviation of the color channel for minimum and maximum range of color histogram
    GlobalColorEstimatorParameter() :
        num_bins (15),
        std_dev_multiplier_(3.f)
    { }

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
        po::options_description desc("Global Color Feature Estimator Parameter\n=====================\n");
        desc.add_options()
                ("help,h", "produce help message")
                ("global_color_num_bins", po::value<size_t>(&num_bins)->default_value(num_bins), "number of bins for each color chanel to create color histogram.")
                ("global_color_std_dev_multiplier", po::value<float>(&std_dev_multiplier_)->default_value(std_dev_multiplier_), "multiplication factor of the standard deviation of the color channel for minimum and maximum range of color histogram.")
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

/**
 * @brief The GlobalColorEstimator class implements a simple global description
 * in terms of the color of the input cloud
 */
template<typename PointT>
class V4R_EXPORTS GlobalColorEstimator : public GlobalEstimator<PointT>
{
private:
    using GlobalEstimator<PointT>::indices_;
    using GlobalEstimator<PointT>::cloud_;
    using GlobalEstimator<PointT>::descr_name_;
    using GlobalEstimator<PointT>::descr_type_;
    using GlobalEstimator<PointT>::feature_dimensions_;

    GlobalColorEstimatorParameter param_;

//    RGB2CIELAB::Ptr color_transf_;

public:
    GlobalColorEstimator(const GlobalColorEstimatorParameter &p = GlobalColorEstimatorParameter())
        : GlobalEstimator<PointT>("global_color", FeatureType::GLOBAL_COLOR),
          param_(p)
    {
        feature_dimensions_ = 3 * param_.num_bins;
    }

    bool compute (Eigen::MatrixXf &signature);

    bool needNormals() const { return false; }

    typedef boost::shared_ptr< GlobalColorEstimator<PointT> > Ptr;
    typedef boost::shared_ptr< GlobalColorEstimator<PointT> const> ConstPtr;
};
}

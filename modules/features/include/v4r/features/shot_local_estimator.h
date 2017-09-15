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
 * @file shot_local_estimator.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Aitor Aldoma (aldoma@acin.tuwien.ac.at)
 * @date 2012
 * @brief
 *
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <v4r/features/local_estimator.h>
#include <v4r/features/types.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

//This stuff is needed to be able to make the SHOT histograms persistent
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<352>, (float[352], histogram, histogram352) )

namespace v4r
{
    class V4R_EXPORTS SHOTLocalEstimationParameter
    {
        public:
        float support_radius_;

        SHOTLocalEstimationParameter() :
            support_radius_ ( 0.05f)
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
            po::options_description desc("SHOT Parameter\n=====================\n");
            desc.add_options()
                    ("help,h", "produce help message")
                    ("shot_support_radius", po::value<float>(&support_radius_)->default_value(support_radius_), "shot support radius")
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
      class V4R_EXPORTS SHOTLocalEstimation : public LocalEstimator<PointT>
      {
      private:
          using LocalEstimator<PointT>::indices_;
          using LocalEstimator<PointT>::cloud_;
          using LocalEstimator<PointT>::normals_;
          using LocalEstimator<PointT>::keypoint_indices_;
          using LocalEstimator<PointT>::descr_name_;
          using LocalEstimator<PointT>::descr_type_;
          using LocalEstimator<PointT>::descr_dims_;

          SHOTLocalEstimationParameter param_;

      public:
        SHOTLocalEstimation ( const SHOTLocalEstimationParameter &p = SHOTLocalEstimationParameter() ):
            param_( p )
        {
            descr_name_ = "shot";
            descr_type_ = FeatureType::SHOT;
            descr_dims_ = 352;
        }

        bool
        acceptsIndices() const
        {
          return true;
        }

        void
        compute(std::vector<std::vector<float> > & signatures);

        bool
        needNormals () const
        {
            return true;
        }

        std::string
                getUniqueId() const
        {
            std::stringstream id;
            id << static_cast<int>( param_.support_radius_ * 1000.f );
            return id.str();
        }

        typedef boost::shared_ptr< SHOTLocalEstimation<PointT> > Ptr;
        typedef boost::shared_ptr< SHOTLocalEstimation<PointT> const> ConstPtr;
      };
}

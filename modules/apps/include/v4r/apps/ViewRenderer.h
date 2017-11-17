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
 * @file ViewRenderer.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at), Simon Schreiberhuber (schreiberhuber@acin.tuwien.ac.at)
 * @date October 2017
 * @brief Renders Views from a textured mesh model and saves it into a colored point cloud.
 *
 */

#pragma once

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <v4r/common/camera.h>
#include <v4r/core/macros.h>
#include <v4r_config.h>

namespace po = boost::program_options;

namespace v4r {

namespace apps {

class V4R_EXPORTS ViewRendererParameter {

 public:
  bool upperHemisphere; //< if true, samples views from the upper hemisphere only (z>0)
  bool autoscale; //< if true, scales the coordinate system such that the object model is enclosed by the unit sphere
  size_t subdivisions;
  float radius;

  ViewRendererParameter()
      : upperHemisphere(false), autoscale(false), subdivisions(0), radius(3.f) {}

/**
* @brief init parameters
* @param command_line_arguments (according to Boost program options library)
* @return unused parameters (given parameters that were not used in this initialization call)
*/
  std::vector<std::string> init(int argc, char **argv) {
    std::vector<std::string> arguments(argv + 1, argv + argc);
    return init(arguments);
  }

/**
 * @brief init parameters
 * @param command_line_arguments (according to Boost program options library)
 * @return unused parameters (given parameters that were not used in this initialization call)
 */
  std::vector<std::string> init(const std::vector<std::string> &command_line_arguments) {

    po::options_description desc(
        "Depth-map and point cloud Rendering from mesh file\n======================================\n**Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "rendering_subdivisions", po::value<size_t>(&subdivisions)->default_value(subdivisions),
        "defines the number of subdivsions used for rendering")(
        "rendering_autoscale", po::bool_switch(&autoscale),
        "scales the model into the unit sphere and translates it to the origin")(
        "rendering_northHemisphere", po::bool_switch(&upperHemisphere),
        "only renders the objects from views of the upper hemisphere")(
        "rendering_radius",
        po::value<float>(&radius)->default_value(radius, boost::str(boost::format("%.2e") % radius)),
        "defines the radius used for rendering");

    po::variables_map vm;
    po::parsed_options parsed =
        po::command_line_parser(command_line_arguments).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      to_pass_further.push_back("-h");
    }
    try {
      po::notify(vm);
    } catch (std::exception &e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    }
    return to_pass_further;
  }

};

/**
 * @brief class to render point cloud from a textured mesh
 */
class V4R_EXPORTS ViewRenderer {
 private:
  const v4r::Camera::ConstPtr cam_;
  ViewRendererParameter param_;

 public:
  ViewRenderer(const Camera::ConstPtr &cam, const ViewRendererParameter &p)
      : cam_(cam), param_(p) {}

  void render(const bf::path &input_path, const bf::path &out_path = "rendered_views");
};

}

}
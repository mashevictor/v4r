//
//  Created by Simon Schreiberhuber on 01.04.15.
//  Copyright (c) 2015 Simon Schreiberhuber. All rights reserved.
//

#include <stdio.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <v4r/io/filesystem.h>
#include <v4r/rendering/depthmapRenderer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <glog/logging.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, const char *argv[]) {
  bf::path input;
  bf::path out_dir = bf::path("tmp") / bf::path("rendered_pointclouds");
  bool visualize = false;
  bool upperHemisphere = false;
  bool autoscale = false;
  bool createNormals = false;
  size_t subdivisions = 0, width = 640, height = 480;
  float radius = 3.0, fx = 535.4, fy = 539.2, cx = 320.1, cy = 247.6;

  google::InitGoogleLogging(argv[0]);

  po::options_description desc(
      "Depth-map and point cloud Rendering from mesh file\n======================================\n**Allowed options");
  desc.add_options()("help,h", "produce help message")("input,i", po::value<bf::path>(&input)->required(),
                                                       "input model (.ply)")(
      "output,o", po::value<bf::path>(&out_dir)->default_value("/tmp/rendered_pointclouds/"),
      "output directory to store the point cloud (.pcd) file")(
      "subdivisions,s", po::value<size_t>(&subdivisions)->default_value(subdivisions),
      "defines the number of subdivsions used for rendering")(
      "autoscale", po::bool_switch(&autoscale),
      "scales the model into the unit sphere and translates it to the origin")(
      "northHemisphere,n", po::bool_switch(&upperHemisphere),
      "only renders the objects from views of the upper hemisphere")(
      "radius,r", po::value<float>(&radius)->default_value(radius, boost::str(boost::format("%.2e") % radius)),
      "defines the radius used for rendering")("width", po::value<size_t>(&width)->default_value(width),
                                               "defines the image width")(
      "normals", po::bool_switch(&createNormals),
      "Creates a pointcloud that also contains the normals")(
      "width", po::value<size_t>(&width)->default_value(width),"defines the image width")(
      "height", po::value<size_t>(&height)->default_value(height), "defines the image height")(
      "fx", po::value<float>(&fx)->default_value(fx, boost::str(boost::format("%.2e") % fx)),
      "defines the focal length in x direction used for rendering")(
      "fy", po::value<float>(&fy)->default_value(fy, boost::str(boost::format("%.2e") % fy)),
      "defines the focal length in y direction used for rendering")(
      "cx", po::value<float>(&cx)->default_value(cx, boost::str(boost::format("%.2e") % cx)),
      "defines the central point of projection in x direction used for rendering")(
      "cy", po::value<float>(&cy)->default_value(cy, boost::str(boost::format("%.2e") % cy)),
      "defines the central point of projection in y direction used for rendering")(
      "visualize,v", po::bool_switch(&visualize), "visualize the rendered depth and color map");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return false;
  }

  try {
    po::notify(vm);
  } catch (std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  CHECK((cx < width) && (cy < height) && (cx > 0) && (cy > 0)) << "Parameters not valid!";

  v4r::DepthmapRenderer renderer(width, height);
  renderer.setIntrinsics(fx, fy, cx, cy);

  v4r::DepthmapRendererModel model(input.string(), "", autoscale);

  if (model.hasColor() || model.hasTexture())
    LOG(INFO) << "Model file has color.";
  else
    LOG(INFO) << "Model file has no color.";

  renderer.setModel(&model);

  std::vector<Eigen::Vector3f> sphere = renderer.createSphere(radius, subdivisions);

  if (upperHemisphere) {
    std::vector<Eigen::Vector3f> upper;
    for (size_t i = 0; i < sphere.size(); i++) {
      if (sphere[i][2] > 0) {
        upper.push_back(sphere[i]);
      }
    }
    sphere = upper;
  }
  LOG(INFO) << "Rendering file " << input;

  if (!sphere.empty())
    v4r::io::createDirIfNotExist(out_dir);

  for (size_t i = 0; i < sphere.size(); i++) {
    // get point from list
    const Eigen::Vector3f &point = sphere[i];
    // get a camera pose looking at the center:
    const Eigen::Matrix4f &orientation = renderer.getPoseLookingToCenterFrom(point);

    renderer.setCamPose(orientation);

    float visible;
    cv::Mat color;
    cv::Mat normal;
    cv::Mat depthmap = renderer.renderDepthmap(visible, color, normal);

    // create and save the according pcd files
    std::stringstream ss;
    ss << "cloud_" << i << ".pcd";

    bf::path output_fn = out_dir / ss.str();
    if (model.hasColor() || model.hasTexture()) {
      if(createNormals){
          const pcl::PointCloud<pcl::PointXYZRGBNormal> cloud = renderer.renderPointcloudColorNormal(visible);
          pcl::io::savePCDFileBinary(output_fn.string(), cloud);
      }else{
        const pcl::PointCloud<pcl::PointXYZRGB> cloud = renderer.renderPointcloudColor(visible);
        pcl::io::savePCDFileBinary(output_fn.string(), cloud);

      }

      if (visualize)
        cv::imshow("color", color);
    } else {
      const pcl::PointCloud<pcl::PointXYZ> cloud = renderer.renderPointcloud(visible);
      pcl::io::savePCDFileBinary(output_fn.string(), cloud);
    }

    LOG(INFO) << "Saved data points to " << output_fn.string() << ".";

    if (visualize) {
      LOG(INFO) << visible << "% visible.";
      cv::imshow("depthmap", depthmap * 0.25);
      cv::imshow("normal",normal*0.5+0.5);
      cv::waitKey();
    }
  }

  return 0;
}

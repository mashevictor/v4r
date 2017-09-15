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
 * @file camera.h
 * @author Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <v4r/io/filesystem.h>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv/cv.h>
#include <fstream>
#include <stdlib.h>
#include <math.h>

namespace v4r
{
/**
 * @brief The Camera class represents a pinhole camera model
 * @author Thomas Faeulhammer
 */
class V4R_EXPORTS Camera
{
protected:
    size_t width_; /// image width in pixel
    size_t height_; /// image height in pixel
    float fx_;   ///< focal length in x
    float fy_;   ///< focal length in y
    float cx_;  ///< central point of projection in x
    float cy_; ///< central point of projection in y
    float horizontal_fov_deg_; ///< camera's horizontal field of view in degree
    cv::Mat_<uchar> camera_depth_registration_mask_; ///< this mask has the same size as the camera image and tells us which pixels can have valid depth pixels and which ones are not seen due to the phsysical displacement between RGB and depth sensor.

    friend class boost::serialization::access;

    template<class Archive> V4R_EXPORTS void serialize(Archive & ar, const unsigned int version)
    {
        (void) version;
        ar      & BOOST_SERIALIZATION_NVP(width_)
                & BOOST_SERIALIZATION_NVP(height_)
                & BOOST_SERIALIZATION_NVP(fx_)
                & BOOST_SERIALIZATION_NVP(fy_)
                & BOOST_SERIALIZATION_NVP(cx_)
                & BOOST_SERIALIZATION_NVP(cy_)
                & BOOST_SERIALIZATION_NVP(horizontal_fov_deg_)
                ;
    }

public:

    typedef boost::shared_ptr< Camera > Ptr;
    typedef boost::shared_ptr< Camera const> ConstPtr;

    Camera(
            float fx = 525.f,
            float fy = 525.f,
            size_t width = 640,
            size_t height = 480,
            float cx = 319.5f,
            float cy = 239.5f,
            float horizontal_fov_deg = 58.f
            )
        :
          width_ (width),
          height_(height),
          fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_ (cy),
          horizontal_fov_deg_ (horizontal_fov_deg)
    {
        camera_depth_registration_mask_ = cv::Mat_<uchar> (height_, width_);
        camera_depth_registration_mask_.setTo(255);
    }

    /**
     * @brief getWidth
     * @return
     */
    size_t getWidth() const { return width_; }

    /**
     * @brief getHeight
     * @return
     */
    size_t getHeight() const { return height_; }

    /**
     * @brief getFocalLength get focal length in x
     * @return focal length in x
     */
    float getFocalLengthX() const { return fx_; }

    /**
     * @brief getFocalLength get focal length along y direction
     * @return focal length in y
     */
    float getFocalLengthY() const { return fy_; }

    /**
     * @brief getCx
     * @return
     */
    float getCx() const { return cx_; }

    /**
     * @brief getCy
     * @return
     */
    float getCy() const { return cy_; }

    /**
     * @brief getHorizontalFOV
     * @return
     */
    float getHorizontalFOV() const { return horizontal_fov_deg_; }

    /**
     * @brief getVerticalFOV
     * @return
     */
    float getVerticalFOV() const
    {
        return 2 * atan( tan( horizontal_fov_deg_ * 0.017453293f / 2.f ) * static_cast<float>(height_) / width_) * 57.29578f;
    }

    /**
     * @brief setCameraDepthRegistrationMask this mask has the same size as the camera image and tells us which pixels can have valid depth pixels and which ones are not seen due to the phsysical displacement between RGB and depth sensor.
     * @param mask valid pixels are set to 255, pixels that are outside depth camera's field of view are set to 0
     */
    void setCameraDepthRegistrationMask( const cv::Mat_<uchar> &mask)
    {
        camera_depth_registration_mask_ = mask;
    }

    /**
     * @brief getCameraDepthRegistrationMask this mask has the same size as the camera image and tells us which pixels can have valid depth pixels and which ones are not seen due to the phsysical displacement between RGB and depth sensor.
     * @return mask valid pixels are set to 255, pixels that are outside depth camera's field of view are set to 0
     */
    cv::Mat_<uchar> getCameraDepthRegistrationMask() const
    {
        return camera_depth_registration_mask_;
    }


    /**
     * @brief save save the parameters into an XML file
     * @param filename
     */
    void
    save(const std::string &filename) const
    {
        std::ofstream ofs(filename);
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp("CameraParameter", *this );
        ofs.close();
    }

    /**
     * @brief Camera initialize the camera with the parameters from an XML file
     * @param filename
     */
    Camera(const std::string &filename)
    {
        if( !v4r::io::existsFile(filename) )
            throw std::runtime_error("Given config file " + filename + " does not exist! Current working directory is " + boost::filesystem::current_path().string() + ".");

        std::ifstream ifs(filename);
        boost::archive::xml_iarchive ia(ifs);
        ia >> boost::serialization::make_nvp("CameraParameter", *this );
        ifs.close();

        camera_depth_registration_mask_ = cv::Mat_<uchar> (height_, width_);
        camera_depth_registration_mask_.setTo(255);
    }
};

}


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
 * @file dmRenderObject.h
 * @author Simon Schreiberhuber (schreiberhuber@acin.tuwien.ac.at)
 * @date Nov, 2015
 * @brief
 *
 */


#ifndef __DM_RENDERER_OBJECT__
#define __DM_RENDERER_OBJECT__


#include <GL/glew.h>
#include <GL/gl.h>


#include <eigen3/Eigen/Eigen>
#include <v4r/core/macros.h>

#include <pcl/PolygonMesh.h>

#include <opencv2/opencv.hpp>


namespace v4r{



class V4R_EXPORTS DepthmapRendererModel{
private:
    friend class DepthmapRenderer;

    struct Vertex;

    Vertex *vertices;
    uint32_t *indices;
    uint32_t vertexCount;
    uint32_t indexCount;



    //TODO: add this!!!!!!!!!
    struct MeshInfo{
        uint32_t beginIndex=0;
        uint32_t indexCount=0;
        //TODO: add texture
        cv::Mat tex;
        GLuint glTex=0;

    };
    std::vector<MeshInfo> meshes;

    float scale;
    Eigen::Vector3f offset;
    bool color;
    bool texture;//TODO add
    bool geometry;

    /**
     * @brief loadToGPU
     *        Uploads geometry data to GPU and returns the OpenGL handles for Vertex and Index Buffers
     * @param VBO
     * @param IBO
     */
    void loadToGPU(GLuint &VBO,GLuint &IBO);

    void unloadFromGPU();

    /**
     * @brief getIndexCount
     * @return
     */
    unsigned int getIndexCount();



public:



    /**
     * @brief DepthmapRendererModel loads the geometry data and creates the necessary opengl ressources
     * @param file filename of the geometry file
     */
    DepthmapRendererModel(const std::string &file, std::string path="", bool shiftToCenterAndNormalizeScale=true);

    /**
     * @brief DepthmapRendererModel loads the geometry data into the opengl context
     * @param pclMesh
     */
    DepthmapRendererModel(const pcl::PolygonMesh &pclMesh, bool shiftToCenterAndNormalizeScale=true);


    DepthmapRendererModel(const DepthmapRendererModel &obj);

    ~DepthmapRendererModel();

    friend void swap(DepthmapRendererModel& first, DepthmapRendererModel& second);

    DepthmapRendererModel& operator =(const DepthmapRendererModel model);






    /**
     * @brief getScale
     * @return the models get fitted into a unity sphere.... Scale and Ofset gives the shift and offset of the object
     * Note: the translation was applied first... so to transform the pointcloud back you have to
     * apply the scale first and and translate afterwards
     */
    float getScale();



    /**
     * @brief hasColor
     * @return
     * true if the loaded mesh contains color information (suitable to generate a colored pointcloud)
     */
    bool hasColor();



    bool hasTexture();


    /**
     * @brief hasGeometry
     * @return
     * true if the geometry file loaded correctly and contains geometry.
     */
    bool hasGeometry();

    /**
     * @brief getOffset
     * @return the models get fitted into a unity sphere.... Scale and Offset gives the shift and offset of the object
     */
    Eigen::Vector3f getOffset();

};

}

#endif /* defined(__DM_RENDERER_OBJECT__) */

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
 * @file StructuralRelations.h
 * @author Richtsfeld
 * @date December 2012
 * @version 0.1
 * @brief Calculate patch relations for structural level: Efficient version without fourier and gabor filter.
 */

#ifndef SURFACE_STRUCTURAL_RELATIONS_LIGHT_H
#define SURFACE_STRUCTURAL_RELATIONS_LIGHT_H

#include <omp.h>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "v4r/attention_segmentation/ColorHistogram.h"
#include "v4r/attention_segmentation/Texture.h"
#include "v4r/attention_segmentation/BoundaryRelationsMeanDepth.h"
#include "v4r/attention_segmentation/BoundaryRelationsMeanColor.h"
#include "v4r/attention_segmentation/BoundaryRelationsMeanCurvature.h"
#include "v4r/attention_segmentation/Fourier.h"
#include "v4r/attention_segmentation/Gabor.h"

#include "v4r/attention_segmentation/SurfaceModel.h"
#include "v4r/attention_segmentation//EPBase.h"

#include "v4r/attention_segmentation/EPUtils.h"


namespace v4r
{
  
class StructuralRelations: public EPBase
{

  enum UsedRelations {
    R_COS   = 0x0001,
    R_TR    = 0x0002,
    R_GS    = 0x0004,
    R_FS    = 0x0008,
    R_RS    = 0x0010,
    R_COS3  = 0x0020,
    R_CUM3  = 0x0040,
    R_DM2   = 0x0080,
    R_DV2   = 0x0100,
    R_CUV3  = 0x0200,
    R_3D2   = 0x0400,
  };
  
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  
  void computeNeighbors();
  
  bool have_surfaces;
  std::vector<SurfaceModel::Ptr> surfaces;              ///< Surfaces
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model;
  
  bool have_neighbours2D, have_neighbours3D;
  std::map<borderIdentification,std::vector<neighboringPair> > ngbr3D_map;
  std::map<borderIdentification,std::vector<neighboringPair> > ngbr2D_map;
  
  std::vector<Relation> surfaceRelations;
  std::vector<Relation> validRelations;
//   bool have_relations;

  int usedRelations;
  bool trainMode;
  bool initialized;
  
  /** Project the datapoints of the plane surfaces to the model surface **/
  void projectPts2Model();

  //for texture
  cv::Mat_<cv::Vec3b> matImage;
  cv::Mat gray_image;
  //
  cv::Mat gray_image2;
  cv::Mat edges;

  std::vector<ColorHistogram::Ptr> hist;
  std::vector<Texture::Ptr> text;
  std::vector<Fourier::Ptr> fourier;
  std::vector<Gabor::Ptr> gabor;
  Gabor::Ptr permanentGabor;
  
public:
  StructuralRelations();
  ~StructuralRelations();
  
  /** Set neighbours 2D **/
  void setNeighbours2D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr2D_map);
  /** Set neighbours 3D **/
  void setNeighbours3D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr3D_map);
  
  /** Set surfaces **/
  void setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces);
  /** Set relations **/
//   void setRelations(const std::vector<Relation> _surfaceRelations);
  /**Set used relations **/
  void setUsedRelations(int _usedRelations);
  /**Set training mode **/
  void setTrainingMode(bool _trainMode);
  
  /** Get surfaces relations **/
//   inline std::vector<Relation> getRelations();
  /** Get surfaces relations **/
  inline std::vector<Relation> getValidRelations();
  /**Get used relations **/
  int getUsedRelations();

  void init();
  /** Compute relations for the segmenter **/
  virtual void compute();
  
};

// inline std::vector<Relation> StructuralRelations::getRelations()
// {
//   return surfaceRelations;
// }

inline std::vector<Relation> StructuralRelations::getValidRelations()
{
  return validRelations;
}

inline int StructuralRelations::getUsedRelations()
{
  return usedRelations;
}

} //--END--

#endif


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

#ifndef SPARSE_MAT_H
#define SPARSE_MAT_H

#include <stdio.h>
#include <map>
#include <vector>

namespace pcl {
namespace on_nurbs {

/** \brief Sparse matrix implementation. */
class SparseMat {
 protected:
  std::map<int, std::map<int, double>> m_mat;

 public:
  /** \brief Constructor clearing memory of matrix (=set to zero) */
  SparseMat() {
    m_mat.clear();
  }

  /** \brief Get a set of values defined by index vectors
   *  \param[in] i vector of row indices
   *  \param[in] j vector of column indices
   *  \param[out] v vector of matrix entries at (i,j)    */
  void get(std::vector<int> &i, std::vector<int> &j, std::vector<double> &v);
  /** \brief Get matrix values at index pair (i,j)
   *  \param[in] i row index
   *  \param[in] j column index
   *  return v matrix value at (i,j)       */
  double get(int i, int j);
  /** \brief Set matrix values at index pair (i,j)
   *  \param[in] i row index
   *  \param[in] j column index
   *  \param[in] v matrix value at (i,j)       */
  void set(int i, int j, double v);

  /** \brief Delete row at index i (= set to zero) */
  void deleteRow(int i);
  /** \brief Delete column at index j (= set to zero) */
  void deleteColumn(int j);

  /** \brief Clear memory of matrix (=set to zero) */
  inline void clear() {
    m_mat.clear();
  }
  /** \brief Get size of matrix with respect to last non-zero entries */
  void size(int &si, int &sj);
  /** \brief Get number of non-zero entries in matrix */
  int nonzeros();
  /** \brief Print all matrix entries with respect to size. */
  void printLong();
  /** \brief Print indices and values of non-zero entries. */
  void print();
};
}
}

#endif /* SPARSEMAT_H_ */

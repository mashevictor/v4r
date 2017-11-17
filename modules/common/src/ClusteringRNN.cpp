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
 * @file main.cpp
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */

/**
 * $Id: ClusteringRNN.cc 51 2014-03-30 12:41:22Z hannes $
 * Johann Prankl, 2011-11-04
 * prankl@acin.tuwien.ac.at
 */

#include <v4r/common/ClusteringRNN.h>

namespace v4r {

using namespace std;

ClusteringRNN::ClusteringRNN(const Parameter &_param, bool _dbg) : param(_param), dbg(_dbg) {}

ClusteringRNN::~ClusteringRNN() {}

/************************************** PRIVATE ************************************/

/************************************** PUBLIC ************************************/

/**
 * find nearest neighbour of CodebookEntries
 */
int ClusteringRNN::getNearestNeighbour(const Cluster &cluster, const std::vector<Cluster::Ptr> &_clusters, float &sim) {
  sim = -FLT_MAX;
  int idx = INT_MAX;
  float tmp;

  for (unsigned i = 0; i < _clusters.size(); i++) {
    tmp = -(cluster.sqr_sigma + _clusters[i]->sqr_sigma + (cluster.data - _clusters[i]->data).squaredNorm());
    if (tmp > sim) {
      sim = tmp;
      idx = i;
    }
  }

  return idx;
}

/**
 * Agglomerate
 */
void ClusteringRNN::agglomerate(const Cluster &src, Cluster &dst) {
  float sum = 1. / (src.indices.size() + dst.indices.size());

  dst.sqr_sigma = sum * (src.indices.size() * src.sqr_sigma + dst.indices.size() * dst.sqr_sigma +
                         sum * src.indices.size() * dst.indices.size() * (src.data - dst.data).squaredNorm());

  // compute new mean model of two clusters
  dst.data *= dst.indices.size();

  dst.data += src.data * src.indices.size();
  dst.data *= sum;

  // add occurrences from c
  dst.indices.insert(dst.indices.end(), src.indices.begin(), src.indices.end());
}

/**
 * initDataStructure
 */
void ClusteringRNN::initDataStructure(const DataMatrix2Df &samples, std::vector<Cluster::Ptr> &data) {
  data.resize(samples.rows);

  for (int i = 0; i < samples.rows; i++) {
    data[i].reset(new Cluster(Eigen::Map<const Eigen::VectorXf>(&samples(i, 0), samples.cols), i));
  }
}

/**
 * create clusters
 */
void ClusteringRNN::cluster(const DataMatrix2Df &samples) {
  int nn, last;
  float sim;
  std::vector<float> lastsim;
  std::vector<Cluster::Ptr> chain;
  std::vector<Cluster::Ptr> remaining;

  initDataStructure(samples, remaining);

  clusters.clear();

  if (remaining.size() == 0)
    return;

  last = 0;
  lastsim.push_back(-FLT_MAX);

  chain.push_back(remaining.back());
  remaining.pop_back();
  float sqrThr = -param.dist_thr * param.dist_thr;

  while (remaining.size() != 0) {
    nn = getNearestNeighbour(*chain[last], remaining, sim);

    if (sim > lastsim[last]) {
      // no RNN -> add to chain
      last++;
      chain.push_back(remaining[nn]);
      remaining.erase(remaining.begin() + nn);
      lastsim.push_back(sim);
    } else {
      // RNN found
      if (lastsim[last] > sqrThr) {
        agglomerate(*chain[last - 1], *chain[last]);
        remaining.push_back(chain[last]);
        chain.pop_back();
        chain.pop_back();
        lastsim.pop_back();
        lastsim.pop_back();
        last -= 2;
      } else {
        // cluster found set codebook
        for (unsigned i = 0; i < chain.size(); i++) {
          clusters.push_back(chain[i]);
        }
        chain.clear();
        lastsim.clear();
        last = -1;
        if (dbg) {
          printf(".");
          fflush(stdout);
        }
      }
    }
    if (remaining.size() == 0) {
      if (lastsim[last] > sqrThr) {
        agglomerate(*chain[last - 1], *chain[last]);
        remaining.push_back(chain[last]);
        chain.pop_back();
        chain.pop_back();
        lastsim.pop_back();
        lastsim.pop_back();
        last -= 2;
      }
    }

    if (last < 0 && remaining.size() > 0) {
      // init new chain
      last++;
      lastsim.push_back(-FLT_MAX);

      chain.push_back(remaining.back());
      remaining.pop_back();
    }
  }

  for (unsigned i = 0; i < chain.size(); i++) {
    clusters.push_back(chain[i]);
  }

  if (dbg)
    cout << endl;
  // if (dbg) cout<<"clusters.size()="<<clusters.size()<<endl;
}

/**
 * getClusters
 */
void ClusteringRNN::getClusters(std::vector<std::vector<int>> &_clusters) {
  _clusters.resize(clusters.size());

  for (unsigned i = 0; i < clusters.size(); i++)
    _clusters[i] = clusters[i]->indices;
}

/**
 * getCenters
 */
void ClusteringRNN::getCenters(DataMatrix2Df &_centers) {
  _centers.clear();

  if (clusters.size() == 0)
    return;

  int cols = clusters[0]->data.size();
  _centers.reserve(clusters.size(), cols);

  for (unsigned i = 0; i < clusters.size(); i++)
    _centers.push_back(&clusters[i]->data[0], cols);
}
}

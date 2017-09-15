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
 * @file train3DEF.cpp
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */
#include <boost/program_options.hpp>

#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_forest.h>

using namespace std;
namespace po = boost::program_options;

int nTrees;
int maxDepth;
float bagging;
int nFeatures;
int nThresholds;
float minGain;
int minPoints;
string forestfile;

bool uniformBags;

string datadir;
string indexfile;
string namesfile;

static bool parseArgs(int argc, char** argv)
{
    po::options_description forest("Forest options");
    forest.add_options()
            ("help,h","")
            ("output,o", po::value<std::string>(&forestfile), "" )
            ("trees,t", po::value<int>(&nTrees)->default_value(60), "Number of trees")
            ("depth,d", po::value<int>(&maxDepth)->default_value(20), "Max. depth of trees (-1 = no limit)")
            ("bagging,b", po::value<float>(&bagging)->default_value(0.3), "Bagging ratio (0-1.0)")
            ("uniformbags,u", po::value<bool>(&uniformBags)->default_value(false), "Uniformly bag training data" )
            ("features,f", po::value<int>(&nFeatures)->default_value(1000), "Max. number of sampled settings per feature")
            ("thresholds,s", po::value<int>(&nThresholds)->default_value(100), "Max. number of sampled thresholds per feature")
            ("gain,g", po::value<float>(&minGain)->default_value(0.02), "Min. gain to split")
            ("points,p", po::value<int>(&minPoints)->default_value(5), "Min. points to split")
            ;

    po::options_description data("Data options");
    data.add_options()
            ("input,i", po::value<std::string>(&datadir), "Input directory of training data")
            ("indexfile,x", po::value<std::string>(&indexfile), "Index file for training data")
            ("namesfile,n", po::value<std::string>(&namesfile), "File containing list of label names")
            ;

    po::options_description all("");
    all.add(data).add(forest);

    po::options_description visible("");
    visible.add(data).add(forest);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).run(), vm);

    po::notify(vm);

    if(vm.count("help") || !vm.count("input") || !vm.count("output") || !vm.count("indexfile") || !vm.count("namesfile"))
    {
        std::cout << "General usage: train3DEF [-options] -o output-file -i trainingdata-dir -x index-file -n classname-file" << std::endl;
        std::cout << visible;
        return false;
    }

    return true;
}

int main (int argc, char** argv)
{
    if(!parseArgs(argc, argv))
        return -1;

    v4r::EntangledForestData d;

    d.LoadTrainingData(datadir, indexfile, namesfile);

    v4r::EntangledForest f(nTrees, maxDepth, bagging, nFeatures, nThresholds, minGain, minPoints);
    f.Train(&d, uniformBags);
    f.SaveToBinaryFile(forestfile);

    return 0;
}

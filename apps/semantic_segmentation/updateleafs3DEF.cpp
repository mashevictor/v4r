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
 * @file updateleafs3DEF.cpp
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */
#include <iostream>
#include <ctime>

#include <opencv2/core/core.hpp>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/program_options.hpp>

#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_forest.h>

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;


string inputfile;
string outputfile;

int depth;
string datadir;
string indexfile;
double updateWeight;

bool bagUniformly;

static bool parseArgs(int argc, char **argv)
{
    po::options_description forest("Options");
    forest.add_options()
            ("help,h", "")
            ("input,i", po::value<string>(&inputfile), "Input forest file")
            ("output,o", po::value<std::string>(&outputfile)->default_value("output.ef"), "Output forest file")
            ("trainingdata,t", po::value<std::string>(&datadir)->default_value("."),
             "Input directory of training data")
            ("indexfile,x", po::value<std::string>(&indexfile)->default_value("indextraining"),
             "Index file of training data")
            ("depth,d", po::value<int>(&depth)->default_value(100), "Depth level to update")
            ("updateweight,u", po::value<double>(&updateWeight)->default_value(1.0),
             "Weight of new distribution compared to old one")
            ("unibags,b", po::value<bool>(&bagUniformly)->default_value(false),
             "Try to uniformly sample training data");

    po::options_description all("");
    all.add(forest);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
            options(all).run(), vm);

    po::notify(vm);

    std::string usage = "General usage: updateleafs -i inputfile -o outputfile";

    if (vm.count("help"))
    {
        std::cout << usage << std::endl;
        std::cout << all;
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    if (!parseArgs(argc, argv))
        return -1;

    cout << "Load forest " << inputfile << endl;
    v4r::EntangledForest f;
    v4r::EntangledForest::LoadFromBinaryFile(inputfile, f);

    cout << "Load training data from " << datadir << endl;
    v4r::EntangledForestData d;
    d.LoadTrainingData(datadir, indexfile);

    // update leaf nodes at certain depth and cut off deeper nodes
    f.updateLeafs(&d, depth, updateWeight, bagUniformly);

    cout << "DONE. Save new forest as " << outputfile << endl;
    f.SaveToBinaryFile(outputfile);
    cout << "DONE" << endl;
}

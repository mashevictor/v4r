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
 * @file merge3DEF.cpp
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date 2017
 * @brief
 *
 */
#include <iostream>
#include <ctime>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <v4r/semantic_segmentation/entangled_data.h>
#include <v4r/semantic_segmentation/entangled_forest.h>

using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;

std::vector<string> forestfiles;
string outputfile;

static bool parseArgs(int argc, char** argv)
{
    po::options_description forest("Options");
    forest.add_options()
            ("help,h","")
            ("forests,f", po::value<std::vector< string > >(&forestfiles)->multitoken(), "Forest files to merge" )
            ("output,o", po::value<std::string>(&outputfile)->default_value("merged.ef"), "" )
            ;

    po::options_description all("");
    all.add(forest);


    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).run(), vm);

    po::notify(vm);

    std::string usage = "General usage: mergeforests -f forest1 forest2 ... -o outputfile";

    if(forestfiles.size() < 2)
    {
        std::cout << "You have to list at least 2 forest files!" << std::endl;
        return false;
    }

    if(vm.count("help"))
    {
        std::cout << usage << std::endl;
        std::cout << all;
        return false;
    }

    return true;
}

int main (int argc, char** argv)
{
    if(!parseArgs(argc, argv))
        return -1;

    cout << "Load forest " << forestfiles[0] << endl;
    v4r::EntangledForest merged;
    v4r::EntangledForest::LoadFromBinaryFile(forestfiles[0], merged);

    for(unsigned int i=1; i < forestfiles.size(); ++i)
    {
        cout << "Merge with forest " << forestfiles[i] << endl;
        v4r::EntangledForest tomerge;
        v4r::EntangledForest::LoadFromBinaryFile(forestfiles[i], tomerge);
        merged.Merge(tomerge);
    }

    cout << "DONE. Save new forest as " << outputfile << endl;
    merged.SaveToBinaryFile(outputfile);
    cout << "DONE"<< endl;
}

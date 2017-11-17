# Contributing to V4R

Please take a moment to review this document in order to make the contribution process easy and effective for everyone involved.

Following these guidelines helps to communicate that you respect the time of the developers managing and developing this open source project. In return, they should reciprocate that respect in addressing your issue or assessingpatches and features.


## Dependencies
V4R is an open-source project with the goal to be easily installed on different platforms by providing released Debian packages for Ubuntu systems. To allow packaging the V4R library, all dependencies need to be defined in `package.xml`. The required names for specific packages can be found [here](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml). Packages not included in this list need to be added as [3rdparty libraries](https://rgit.acin.tuwien.ac.at/v4r/v4r_internal/wikis/how-to-add-third-party-dependency) to V4R. Whenever possible, try to depend on packaged libraries. This especially applies to PCL and OpenCV. Currently this means contribute your code such that it is compatible to PCL 1.7.2 and OpenCV 2.4.9.  
Also, even though V4R stands for Vision for Robotics, our library is independent of ROS. If you need a ROS component, put your core algorithms into this V4R library and create wrapper interfaces in the seperate [v4r_ros_wrappers repository](https://rgit.acin.tuwien.ac.at/v4r/v4r_ros_wrappers).

## Using the issue tracker

The issue tracker for [internal](https://rgit.acin.tuwien.ac.at/v4r/v4r_internal/issues) or [public](https://rgit.acin.tuwien.ac.at/v4r/v4r/issues) issues are the preferred channel for submitting [pull requests](#pull-requests) and [bug reports](#bugs). Please **do not** derail or troll issues. Keep the discussion on topic and  respect the opinions of others.


<a name="pull-requests"></a>
## Pull requests

Pull requests let you tell others about changes you've pushed to a repository on GitHub. Once a pull request is sent, interested parties can review the set of changes, discuss potential modifications, and even push follow-up commits if necessary. Therefore, this is the preferred way of pushing your changes - **do not** push your changes directly onto the master branch!
Also, keep your pull requests small and focussed on a specific issue/feature. Do not accumulate months of changes into a single pull request! Such a pull request can not be reviewed!

Good pull requests - patches, improvements, new features - are a fantastic help. They should remain focused in scope and avoid containing unrelated commits.


<a name="checklist"></a>
### Checklist

Please use the following checklist to make sure that your contribution is well
prepared for merging into V4R:

1. Source code adheres to the coding conventions described in [V4R Style Guide](docs/v4r_style_guide.md).
   But if you modify existing code, do not change/fix style in the lines that
   are not related to your contribution.

2. Commit history is tidy (no merge commits, commits are [squashed](http://davidwalsh.name/squash-commits-git)
   into logical units).

3. Each contributed file has a [license](#license) text on top.


<a name="bugs"></a>
## Bug reports

A bug is a _demonstrable problem_ that is caused by the code in the repository.
Good bug reports are extremely helpful - thank you!

Guidelines for bug reports:

1. **Check if the issue has been reported** &mdash; use issue search.

2. **Check if the issue has been fixed** &mdash; try to reproduce it using the  latest `master` branch in the repository.

3. **Isolate the problem** &mdash; ideally create a reduced test
   case.

A good bug report shouldn't leave others needing to chase you up for more
information. Please try to be as detailed as possible in your report. What is
your environment? What steps will reproduce the issue? What would you expect to
be the outcome? All these details will help people to fix any potential bugs.
After the report of a bug, a responsible person will be selected and informed to solve the issue.

<a name="license"></a>
## License

V4R is dual-licensed as GPLv3 and a commercial license (see [this](./LICENSE) for details). By contributing to the V4R repository in any way such as by submitting patches, enhancements or new code modules, you agree to fully transfer the copyright to TU Wien, ACIN, V4R and license your work under the stated [Terms and Licenses](./LICENSE). The corpus of the license should be inserted as a C++ comment on top of each source file such as `.h`, `.hpp` and `.cpp` files:

```cpp
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
****************************************************************************
```

As V4R additionally grants the authors of the files to use **their** code contribution without any restrictions, please indicate your modifications and contributions accordingly. For instance. if you are author of a new file, please add the doxygen keywords for `@author`, `@date`, `@file` and a `@brief` description of the file before the actual code part.

<a name="structure"></a>
## Structure
The repostiory consists of several folders and files containing specific parts of the library. This section gives a short introduction to the most important ones.

### ./3rdparty
See Dependencies.

### ./apps
Bigger code examples and tools such as RTMT. Apps typically depend on one or multiple modules.

### ./cmake
Several cmake macros.

### ./docs
Tutorials and documentations for various V4R apps and components.

### ./modules
Contains all core components of the library and is organized in logical sub folders which are further called 'packages'.
A package holds the source files which are located in './src'. 
The corresponding header files are located in './include/v4r/package_name/'

By following this structure, new modules can be easily added to the corresponding `CMakeLists.txt`with
```cpp
v4r_define_module(package_name REQUIRED components)
```
i.e. 
```cpp
v4r_define_module(change_detection REQUIRED v4r_common pcl opencv)
```

* `./modules/common` &mdash; anything not specific to a particular module but can (potentially) be reused by multiple packages
* `./modules/core` &mdash; core is used by every module and does only include macros. 

To make your modules visible to other modules you have to add the macros and `V4R_EXPORTS` to your header files. 
```cpp
#include <v4r/core/macros.h>
...
class V4R_EXPORTS ...
```

### samples 
* `./samples/exsamples`&mdash;  short code pieces that demonsrate how to use a module.
* `./samples/tools`&mdash; small tools with only one file

### Other Files
**AUTHORS** &mdash; list of all authors that contributed to V4R. Please insert your name and e-mail address if you add a contribution and are okay to be contacted in case of questions.

**CITATION.md** &mdash; This files includes bibTex encoded references. They can be used to cite the appropriate modules if you use V4R in your work.

**CONTRIBUTING.md** &mdash; The file you read at the moment.


<a name="Documentation"></a>
## Documentation
**ALWAYS DOCUMENT your code!!!** We use Doxygen. A nice introduction to Doxygen can be found [here](https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html).

The Doxygen documentation has to be compiled localy on your system at the moment.

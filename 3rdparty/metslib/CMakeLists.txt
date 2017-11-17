cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(METSlib)

install(DIRECTORY .
        DESTINATION "include/metslib"
        FILES_MATCHING PATTERN "*.hh"
)

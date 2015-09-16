CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)

PROJECT(mbsimEHD C CXX)

### The version number
SET (mbsimEHD_VERSION_MAJOR 0)
SET (mbsimEHD_VERSION_MINOR 1)

#############################################
### Options
#############################################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif(COMPILER_SUPPORTS_CXX11)

#############################################
### Search for dependencies
#############################################
find_package(PkgConfig) # Use PkgConfig for searching
pkg_check_modules(MBSIM mbsim>=4.0.0)

# Add for linkage
include_directories(${MBSIM_INCLUDE_DIRS})
link_directories(${MBSIM_LIBRARY_DIRS})

#############################################
### Include the subdirectories
#############################################

add_subdirectory(mbsimEHD)

#############################################
### Set some properties
#############################################
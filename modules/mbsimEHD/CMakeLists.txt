CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)

PROJECT(mbsimEHD C CXX)

### The version number
SET (mbsimEHD_VERSION_MAJOR 0)
SET (mbsimEHD_VERSION_MINOR 1)

#############################################
### Options
#############################################
OPTION(BUILD_mbsimEHD_SHARED "Build the libraries shared." ON)

IF(BUILD_mbsimEHD_SHARED)
  SET(mbsimEHD_LIBRARY_TYPE SHARED)
ELSE(BUILD_mbsimEHD_SHARED)
  SET(mbsimEHD_LIBRARY_TYPE STATIC)
ENDIF(BUILD_mbsimEHD_SHARED)

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
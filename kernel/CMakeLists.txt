CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)

PROJECT(mbsim Fortran C CXX)

### The version number
SET (mbsim_MAJOR_VERSION 4)
SET (mbsim_MINOR_VERSION 0)
SET (mbsim_PATCH_VERSION 0)
set(mbsim_VERSION ${mbsim_MAJOR_VERSION}.${mbsim_MINOR_VERSION}.${mbsim_PATCH_VERSION})

#############################################
### Options #################################
#############################################

# make sure that the default is a RELEASE-build
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RELEASE CACHE STRING
      "Choose the type of build, options are: None Debug Release."
      FORCE)
endif (NOT CMAKE_BUILD_TYPE)

### For the Fortran wrapping and mangeling
include(FortranCInterface)
FortranCInterface_HEADER(FC.h MACRO_NAMESPACE "FC_")
FortranCInterface_VERIFY(CXX QUIET)

#############################################
### Search for dependencies #################
#############################################
### Use PkgConfig for searching
include(FindPkgConfig) 

### Serach for openmbvcppinterface
pkg_check_modules(OPENMBVCPPINTERFACE REQUIRED openmbvcppinterface>=3.1.0)
include_directories(${OPENMBVCPPINTERFACE_INCLUDE_DIRS})
string (REPLACE ";" " " OPENMBVCPPINTERFACE_CFLAGS_STR "${OPENMBVCPPINTERFACE_CFLAGS}")

# Check for ANSICSIGNAL
INCLUDE (CheckIncludeFiles)
check_include_files("signal.h" HAVE_ANSICSIGNAL)

#############################################
### Include the subdirectories ##############
#############################################
add_subdirectory(mbsim)

#############################################
###  Installing #############################
#############################################
###  Install the pkg-config file
configure_file("mbsim.pc.in" "mbsim.pc" @ONLY)
install(FILES       "${CMAKE_CURRENT_BINARY_DIR}/mbsim.pc"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
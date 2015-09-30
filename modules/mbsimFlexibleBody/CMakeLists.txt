CMAKE_MINIMUM_REQUIRED (VERSION 2.8.12)

PROJECT(mbsimFlexibleBody)

### The version number
SET (mbsimFlexibleBody_MAJOR_VERSION 0)
SET (mbsimFlexibleBody_MINOR_VERSION 2)
SET (mbsimFlexibleBody_PATCH_VERSION 0)
set(mbsimFlexibleBody_VERSION ${mbsimFlexibleBody_MAJOR_VERSION}.${mbsimFlexibleBody_MINOR_VERSION}.${mbsimFlexibleBody_PATCH_VERSION})

#############################################
### Options #################################
#############################################

### Use features of c++11 (to avoid warnings only)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else(COMPILER_SUPPORTS_CXX11)
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif(COMPILER_SUPPORTS_CXX11)

#############################################
### Search for dependencies #################
#############################################

### Use PkgConfig for searching
include(FindPkgConfig) 
pkg_check_modules(MBSIM REQUIRED mbsim>=4.0.0)

### mbsim
# Add for linkage: the include directories as well as the link-directories 
include_directories(${MBSIM_INCLUDE_DIRS})
link_directories(${MBSIM_LIBRARY_DIRS})
# prepare the cflags for the build-process --> the *_CFLAGS-variable comes as a semicolon-separated list, which has to bea space-separated list  
string (REPLACE ";" " " MBSIM_CFLAGS_STR "${MBSIM_CFLAGS}")

#############################################
### Include the subdirectories ##############
#############################################
add_subdirectory(mbsimFlexibleBody)

#############################################
###  Installing #############################
#############################################
###  Install the pkg-config file
configure_file("mbsimFlexibleBody.pc.in" "mbsimFlexibleBody.pc" @ONLY)
install(FILES       "${CMAKE_CURRENT_BINARY_DIR}/mbsimFlexibleBody.pc"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
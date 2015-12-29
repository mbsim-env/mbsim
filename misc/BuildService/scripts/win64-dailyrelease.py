#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import time
import email
import subprocess
import shutil

SRCDIR="/home/mbsim/win64-dailyrelease"
OUTDIR="/var/www/html/mbsim/win64-dailyrelease"
URL="http://www.mbsim-env.de/mbsim/win64-dailyrelease"
SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))


os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-win64/lib/pkgconfig:/home/mbsim/3rdparty/coin-local-win64/lib/pkgconfig:/usr/x86_64-w64-mingw32/sys-root/mingw/lib/pkgconfig:/home/mbsim/3rdparty/xerces-c-local-win64/lib/pkgconfig:/home/mbsim/3rdparty/libarchive-local-win64/lib/pkgconfig"
os.environ["WINEPATH"]="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/home/mbsim/3rdparty/lapack-local-win64/bin"
os.environ["CXXFLAGS"]="-g -O2"
os.environ["CFLAGS"]="-g -O2"
os.environ["FFLAGS"]="-g -O2"
os.environ["PLATFORM"]="Windows" # required for source code examples
os.environ["CXX"]="x86_64-w64-mingw32-g++" # required for source code examples

#mfmf remove forcebuild
if subprocess.call([SCRIPTDIR+"/build.py", "--rotate", "14", "-j", "2", "--sourceDir", SRCDIR, "--prefix", SRCDIR+"/local", "--reportOutDir", OUTDIR+"/report", "--url", URL+"/report", "--buildType", "win64-dailyrelease", "--passToConfigure", "--enable-shared", "--disable-static", "--build=x86_64-redhat-linux", "--host=x86_64-w64-mingw32", "--with-javajnicflags=-I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.37.x86_64/include -I/home/mbsim/SCRIPTS/mbsim/misc/BuildService/scripts/buildPreparation/windows", "--with-mkoctfile=/home/mbsim/3rdparty/octave-local-win64/bin/mkoctfile.exe", "--with-hdf5-prefix=/home/mbsim/3rdparty/hdf5-local-win64", "--with-lapack-lib-prefix=/home/mbsim/3rdparty/lapack-local-win64/lib", "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin", "--passToRunexamples", "--disableCompare", "--disableValidate", "--exeExt", ".exe", "xmlflat/hierachical_modelling", "xml/hierachical_modelling", "xml/time_dependent_kinematics", "xml/hydraulics_ballcheckvalve", "fmi/simple_test", "fmi/hierachical_modelling", "fmi/sphere_on_plane", "mechanics/basics/hierachical_modelling", "mechanics/basics/time_dependent_kinematics"])!=0:
#--with-qwt-inc-prefix=/usr/i686-w64-mingw32/sys-root/mingw/include/qwt --with-qwt-lib-name=qwt5 --with-boost-thread-lib=boost_thread-gcc47-mt-1_48 --with-boost-timer-lib=boost_timer-gcc47-1_48 --with-boost-chrono-lib=boost_chrono-gcc47-1_48 --with-boost-filesystem-lib=boost_filesystem-gcc47-1_48 --with-boost-system-lib=boost_system-gcc47-1_48 --with-boost-locale-lib=boost_locale-gcc47-1_48 --with-boost-date-time-lib=boost_date_time-gcc47-1_48 --with-windres=i686-w64-mingw32-windres
  print("win64-dailyrelease failed.")

#mfmf f=open(OUTDIR+"/report_distribute/distribute.out", "w")
#mfmf if subprocess.call([SCRIPTDIR+"/win64-dailyrelease-distribute.sh"], stderr=subprocess.STDOUT, stdout=f)!=0:
#mfmf   import addBuildSystemFeed
#mfmf   addBuildSystemFeed.add("win64-dailyrelease-distribution", "Distribution: win64-dailyrelease",
#mfmf                          "Unable to create the binary distribution file.", URL+"/report_distribute/distribute.out")
#mfmf f.close()
#mfmf 
#mfmf shutil.copy(SRCDIR+"/dist_mbsim/mbsim-env-win64-shared-build-xxx.zip", OUTDIR+"/download/")
#mfmf shutil.copy(SRCDIR+"/dist_mbsim/mbsim-env-win64-shared-build-xxx-debug.zip", OUTDIR+"/download/")

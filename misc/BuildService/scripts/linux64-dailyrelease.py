#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import time
import email
import subprocess
import simplesandbox
import shutil
import buildSystemState

SRCDIR="/home/mbsim/linux64-dailyrelease"
OUTDIR="/var/www/html/mbsim/linux64-dailyrelease"
URL="http://www.mbsim-env.de/mbsim/linux64-dailyrelease"
SCRIPTDIR=os.path.dirname(os.path.realpath(__file__))


os.environ["PKG_CONFIG_PATH"]=SRCDIR+"/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-linux64/lib/pkgconfig:"+ \
  "/home/mbsim/3rdparty/coin-local-linux64/lib/pkgconfig"
os.environ["LD_LIBRARY_PATH"]="/home/mbsim/3rdparty/casadi-local-linux64/lib"
os.environ["CXXFLAGS"]="-g -O2"
os.environ["CFLAGS"]="-g -O2"
os.environ["FFLAGS"]="-g -O2"

if subprocess.call([SCRIPTDIR+"/build.py", "--buildSystemRun", "--rotate", "14", "-j", "2", "--sourceDir", SRCDIR,
  "--prefix", SRCDIR+"/local", "--reportOutDir", OUTDIR+"/report", "--url", URL+"/report", "--buildType", "linux64-dailyrelease",
  "--enableCleanPrefix", "--passToConfigure", "--enable-shared", "--disable-static", "--with-qwt-inc-prefix=/usr/include/qwt",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin",
  "--with-javajnicflags=-I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.37.x86_64/include -I/usr/lib/jvm/java-1.6.0-openjdk-1.6.0.37.x86_64/include/linux",
  "--passToRunexamples", "--disableCompare", "--disableValidate", "xmlflat/hierachical_modelling", "xml/hierachical_modelling",
  "xml/time_dependent_kinematics", "xml/hydraulics_ballcheckvalve", "fmi/simple_test", "fmi/hierachical_modelling", "fmi/sphere_on_plane",
  "mechanics/basics/hierachical_modelling", "mechanics/basics/time_dependent_kinematics"])!=0:
  print("linux64-dailyrelease failed.")

buildNr=os.readlink(OUTDIR+"/report/result_current")
os.mkdir(OUTDIR+"/report/"+buildNr+"/distribute")
f=open(OUTDIR+"/report/"+buildNr+"/distribute/log.txt", "w")
ret=simplesandbox.call([SCRIPTDIR+"/distribute.py", "/home/mbsim/linux64-dailyrelease/local", "--distFile",
                       OUTDIR+"/report/"+buildNr+"/distribute/mbsim-env-linux64-shared-build-xxx"],
                       shareddir=[OUTDIR+"/report/"+buildNr+"/distribute"],
                       buildSystemRun=True,
                       stderr=subprocess.STDOUT, stdout=f)
buildSystemState.update("linux64-dailyrelease-distribution", "Distribution Failed: linux64-dailyrelease",
                        "Unable to create the binary distribution file.", URL+"/report/"+buildNr+"/distribute/log.txt",
                        0 if ret==0 else 1, 1)
f.close()

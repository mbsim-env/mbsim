#!/usr/bin/python

import subprocess
import os
import fcntl
import json
import time
import sys

# run only one instance of this program at the same time
running=map(lambda x: int(x), subprocess.check_output(["/usr/sbin/pidof", "-x", os.path.basename(__file__)]).split()) # get all this processes
running.remove(os.getpid()) # remove this process itself
if len(running)>0: # to nothing if a process is already running
  sys.exit(0)



scriptdir=os.path.dirname(os.path.realpath(__file__))
SRCDIR="/home/mbsim/linux64-ci"

# set build enironment
env=os.environ.copy()
env['CXXFLAGS']='-O0 -g'
env['CFLAGS']='-O0 -g'
env['FFLAGS']='-O0 -g'
env['PKG_CONFIG_PATH']=SRCDIR+'/local/lib/pkgconfig:/home/mbsim/3rdparty/casadi-local-linux64/lib/pkgconfig:'+\
                       '/home/mbsim/3rdparty/coin-local-linux64/lib/pkgconfig'
env['LD_LIBRARY_PATH']='/home/mbsim/3rdparty/casadi-local-linux64/lib'

# config files
configFilename="/home/mbsim/BuildServiceConfig/mbsimBuildService.conf"

def checkToBuild(tobuild):
  # read file config file
  fd=open(configFilename, 'r+')
  fcntl.lockf(fd, fcntl.LOCK_EX)
  config=json.load(fd)
  
  # get branch to build
  if tobuild==None and len(config['tobuild'])>0:
    tobuild=config['tobuild'][0]
  newTobuild=[]
  for b in config['tobuild']:
    if tobuild['fmatvec']==b['fmatvec'] and \
       tobuild['hdf5serie']==b['hdf5serie'] and \
       tobuild['openmbv']==b['openmbv'] and \
       tobuild['mbsim']==b['mbsim']:
      tobuild=b
    else:
      newTobuild.append(b)
  config['tobuild']=newTobuild
  
  # write file config file
  fd.seek(0);
  json.dump(config, fd, indent=2)
  fd.truncate();
  fcntl.lockf(fd, fcntl.LOCK_UN)
  fd.close()

  return tobuild

tobuild=checkToBuild(None)

if tobuild==None:
  # nothing to do, return with code 0
  sys.exit(0)

# set branches
fmatvecBranch=tobuild['fmatvec'].encode('utf-8')
hdf5serieBranch=tobuild['hdf5serie'].encode('utf-8')
openmbvBranch=tobuild['openmbv'].encode('utf-8')
mbsimBranch=tobuild['mbsim'].encode('utf-8')

# wait at least 2 minutes after the timestamp to give the user the chance to update also other repos
while True:
  delta=tobuild['timestamp']+1*60 - time.time()
  if delta>0:
    time.sleep(delta)
    tobuild=checkToBuild(tobuild)
  else:
    break

if subprocess.call([scriptdir+"/build.py", "--buildSystemRun", "--rotate", "30", "-j", "2", "--fmatvecBranch", fmatvecBranch,
  "--hdf5serieBranch", hdf5serieBranch, "--openmbvBranch", openmbvBranch, "--mbsimBranch", mbsimBranch, "--enableCleanPrefix",
  "--disableConfigure", "--disableMakeClean", "--disableDoxygen", "--disableXMLDoc", "--sourceDir", SRCDIR,
  "--prefix", SRCDIR+"/local", "--reportOutDir", "/var/www/html/mbsim/linux64-ci/report",
  "--url", "http://www.mbsim-env.de/mbsim/linux64-ci/report", "--buildType", "linux64-ci", "--passToConfigure", "--enable-debug",
  "--enable-shared", "--disable-static", "--with-qwt-inc-prefix=/usr/include/qwt", "--with-qmake=qmake-qt4",
  "--with-swigpath=/home/mbsim/3rdparty/swig-local-linux64/bin",
  "--passToRunexamples", "--disableCompare", "--disableMakeClean", "xmlflat/hierachical_modelling", "xml/hierachical_modelling",
  "xml/time_dependent_kinematics", "xml/hydraulics_ballcheckvalve", "fmi/simple_test", "fmi/hierachical_modelling", "fmi/sphere_on_plane",
  "mechanics/basics/hierachical_modelling", "mechanics/basics/time_dependent_kinematics"], env=env)!=0:
  print("CI Build failed.")

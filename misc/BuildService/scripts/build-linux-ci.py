#!/usr/bin/python

import subprocess
import os
import fcntl
import json
import time
import sys

# set build enironment
env=os.environ.copy()
env['CXXFLAGS']='-O0 -g'
env['CFLAGS']='-O0 -g'
env['INSTALL']='/usr/bin/install -c -p'
env['PKG_CONFIG_PATH']='/home/user/3rdparty/casadi-local-linux32/lib/pkgconfig'
env['LD_LIBRARY_PATH']=env.get('LD_LIBRARY_PATH', '')+':/home/user/3rdparty/casadi-local-linux32/lib'

# config files
scriptdir=os.path.dirname(__file__)
configFilename="/home/user/BuildServiceConfig/mbsimBuildService.conf"

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
  json.dump(config, fd)
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
  delta=tobuild['timestamp']+2*60 - time.time()
  if delta>0:
    time.sleep(delta)
    tobuild=checkToBuild(tobuild)
  else:
    break

subprocess.check_call([scriptdir+"/build.py", "--rotate", "30", "-j", "2", "--fmatvecBranch", fmatvecBranch, "--hdf5serieBranch", hdf5serieBranch, "--openmbvBranch", openmbvBranch, "--mbsimBranch", mbsimBranch, "--disableConfigure", "--disableMakeClean", "--disableDoxygen", "--disableXMLDoc", "--sourceDir", "/home/user/MBSimContinuousIntegration", "--prefix", "/home/user/MBSimContinuousIntegration/local", "--reportOutDir", "/var/www/html/mbsim-env/MBSimContinuousIntegration/report", "--url", "http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimContinuousIntegration/report", "--buildType", "Continuous Integration Build:", "--passToConfigure", "--enable-debug", "--enable-shared", "--disable-static", "--with-qwt-inc-prefix=/usr/include/qwt", "--with-boost-locale-lib=boost_locale-mt", "--with-swigpath=/home/user/Updates/local/bin", "--passToRunexamples", "--disableCompare", "--disableMakeClean", "xmlflat/hierachical_modelling", "xml/hierachical_modelling", "xml/time_dependent_kinematics", "xml/hydraulics_ballcheckvalve", "fmi/simple_test", "fmi/hierachical_modelling", "fmi/sphere_on_plane", "mechanics/basics/hierachical_modelling", "mechanics/basics/time_dependent_kinematics"], env=env)
# build done, return with code 1 or other code !=0 returned from subprocess.check_call exception
sys.exit(1)

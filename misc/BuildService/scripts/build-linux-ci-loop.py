#!/usr/bin/python

import subprocess
import os
import fcntl
import json
import time

# set build enironment
env=os.environ.copy()
env['CXXFLAGS']='-O0 -g'
env['CFLAGS']='-O0 -g'
env['INSTALL']='/usr/bin/install -c -p'

# config files
scriptdir=os.path.dirname(__file__)
configFilename="/home/user/BuildServiceConfig/mbsimBuildService.conf"

def buildCI():
  # read file config file
  fd=open(configFilename, 'r+')
  fcntl.lockf(fd, fcntl.LOCK_EX)
  config=json.load(fd)
  
  # get branch to build
  tobuild=None
  newTobuild=[]
  for b in config['tobuild']:
    if config['tobuild'][0]['fmatvec']==b['fmatvec'] and \
       config['tobuild'][0]['hdf5serie']==b['hdf5serie'] and \
       config['tobuild'][0]['openmbv']==b['openmbv'] and \
       config['tobuild'][0]['mbsim']==b['mbsim']:
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

  if tobuild==None:
    return
  
  # set branches
  fmatvecBranch=tobuild['fmatvec'].encode('utf-8')
  hdf5serieBranch=tobuild['hdf5serie'].encode('utf-8')
  openmbvBranch=tobuild['openmbv'].encode('utf-8')
  mbsimBranch=tobuild['mbsim'].encode('utf-8')
  
  # wait at least 2 minutes after the timestamp
  while tobuild['timestamp']+2*60>time.time():
    time.sleep(10)
  
  try:
    subprocess.check_call([scriptdir+"/build.py", "--rotate", "30", "-j", "2", "--fmatvecBranch", fmatvecBranch, "--hdf5serieBranch", hdf5serieBranch, "--openmbvBranch", openmbvBranch, "--mbsimBranch", mbsimBranch, "--disableConfigure", "--disableMakeClean", "--disableDoxygen", "--disableXMLDoc", "--sourceDir", "/home/user/MBSimContinuousIntegration", "--prefix", "/home/user/MBSimContinuousIntegration/local", "--reportOutDir", "/var/www/html/mbsim-env/MBSimContinuousIntegration/report", "--url", "http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimContinuousIntegration/report", "--buildType", "Continuous Integration Build:", "--passToConfigure", "--enable-debug", "--enable-shared", "--disable-static", "--with-qwt-inc-prefix=/usr/include/qwt", "--with-boost-locale-lib=boost_locale-mt", "--with-swigpath=/home/user/Updates/local/bin", "--passToRunexamples", "--disableCompare", "--disableMakeClean", "xmlflat/hierachical_modelling", "xml/hierachical_modelling", "xml/time_dependent_kinematics", "xml/hydraulics_ballcheckvalve", "fmi/simple_test", "fmi/hierachical_modelling", "fmi/sphere_on_plane", "mechanics/basics/hierachical_modelling", "mechanics/basics/time_dependent_kinematics"], env=env)
  except subprocess.CalledProcessError:
    pass


while True:
  buildCI()
  time.sleep(60)

#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import os
import glob
import simplesandbox
import addBuildSystemFeed
import subprocess
import datetime

scriptdir=os.path.dirname(os.path.realpath(__file__))

os.chdir("/home/mbsim/linux64-dailydebug/mbsim/misc/html/doc")
curdir=os.getcwd()

nrDocFailed=0
f=open("/var/www/html/mbsim/doc_manualsbuild.log", "w")
print("Logfile of the build process of the manuals. Generated on "+str(datetime.datetime.now()), file=f)
print("", file=f)
f.flush()
for texMain in glob.glob("*/main.tex"):
  os.chdir(os.path.dirname(texMain))

  if simplesandbox.call([scriptdir+"/builddocsb.py"], shareddir=["."], stderr=subprocess.STDOUT, stdout=f)!=0:
    nrDocFailed+=1
  f.flush()

  os.chdir(curdir)
f.close()

if nrDocFailed>0:
  addBuildSystemFeed.add("build-manuals", "Building Manuals Failed", str(nrDocFailed)+" of "+str(len(glob.glob("*/main.tex")))+" manuals failed to build.",
                         "http://www.mbsim-env.de/mbsim/doc/")

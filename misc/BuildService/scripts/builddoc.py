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

failed=False
f=open("/var/www/html/mbsim/doc_manualsbuild.log", "w")
print("Logfile of the build process of the manuals. Generated on "+str(datetime.datetime.now()), file=f)
print("", file=f)
f.flush()
for texMain in glob.glob("*/main.tex"):
  os.chdir(os.path.dirname(texMain))

  if simplesandbox.call([scriptdir+"/builddocsb.py"], shareddir=["."], stderr=subprocess.STDOUT, stdout=f)!=0:
    failed=True
  f.flush()

  os.chdir(curdir)
f.close()

if failed:
  addBuildSystemFeed.add("manualdoc", "Building Manuals Failed", "Building the latex manuals failed.",
                         "http://www.mbsim-env.de/mbsim/doc/")

#!/usr/bin/python

import os
import glob
import simplesandbox

scriptdir=os.path.dirname(os.path.realpath(__file__))

os.chdir("/home/mbsim/linux64-dailydebug/mbsim/misc/html/doc")
curdir=os.getcwd()

for texMain in glob.glob("*/main.tex"):
  os.chdir(os.path.dirname(texMain))

  print(__file__)
  if simplesandbox.call([scriptdir+"/builddocsb.py"], shareddir=["."])!=0:
    print("Generating pdf for "+os.getcwd()+" failed.")

  os.chdir(curdir)

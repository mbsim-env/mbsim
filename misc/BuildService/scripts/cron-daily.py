#!/usr/bin/python

import subprocess
import os

scriptdir=os.path.dirname(os.path.realpath(__file__))

subprocess.call([scriptdir+"/linux64-dailydebug.py"])

subprocess.call([scriptdir+"/builddoc.py"])

subprocess.call([scriptdir+"/linux64-dailyrelease.sh"])

#subprocess.call([scriptdir+"/win64-dailyrelease.sh"])

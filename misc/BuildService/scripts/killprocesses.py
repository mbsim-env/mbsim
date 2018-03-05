#!/usr/bin/python

import subprocess
import datetime
import re
import os
import signal

def killCrashedProcess(name, timedelta):
  try:
    for pid in subprocess.check_output(["/usr/sbin/pidof", "-x", name]).split():
      startTime=datetime.datetime.strptime(subprocess.check_output(["ps", "-p", pid, "-o", "lstart="]).rstrip(), "%c")
      curTime=datetime.datetime.now()
      if curTime-startTime>timedelta:
        print("Killing program "+name+" with pid "+pid+" and all childs.")
        line=subprocess.check_output(["pstree", "-lAp", pid]).rstrip()
        for p in reversed(re.sub("[^(]*\(([0-9]+)\)", r'\1 ', line).split()):
          os.kill(int(p), signal.SIGTERM)
        time.sleep(10)
        for p in reversed(re.sub("[^(]*\(([0-9]+)\)", r'\1 ', line).split()):
          os.kill(int(p), 9)
  except:
    pass

killCrashedProcess("linux64-dailydebug.py", datetime.timedelta(hours=9))
killCrashedProcess("linux64-dailyrelease.py", datetime.timedelta(hours=3))
killCrashedProcess("win64-dailyrelease.sh", datetime.timedelta(hours=3))

killCrashedProcess("linux64-ci.py", datetime.timedelta(hours=2))

killCrashedProcess("builddoc.py", datetime.timedelta(hours=1))

#!/usr/bin/python

import os
import signal
import subprocess

sbuser="mbsimsb" # username of the sandboxed user
sbuserID="1001" # user id of the sandboxed user
userID="1000" # user id of the user using the sandbox

sigStore={}
shareddirStore=[]

def cleanup(shareddir=None):
  signal.signal(signal.SIGINT, sigStore['INT'])
  signal.signal(signal.SIGTERM, sigStore['TERM'])
  signal.signal(signal.SIGHUP, sigStore['HUP'])
  signal.signal(signal.SIGQUIT, sigStore['QUIT'])
  # fix permissions (as the local user)
  if shareddir!=None and len(shareddir)>0:
    subprocess.check_call([os.path.dirname(__file__)+"/chown_mbsim_mbsimsb", userID]+shareddir)
  if shareddir==None and len(shareddirStore)>0:
    subprocess.check_call([os.path.dirname(__file__)+"/chown_mbsim_mbsimsb", userID]+shareddirStore)

def handler(signum, frame):
  print('Recived signal '+str(signum)+': cleanup and reraise signal.')
  cleanup()
  os.kill(os.getpid(), signum)

def call(cmd, envvar=[], shareddir=[], stdout=None, stderr=None, buildSystemRun=False):
  # normal call (WITHOUT a simple sandbox)
  if not buildSystemRun:
    return subprocess.call(cmd, stdout=stdout, stderr=stderr)

  # simple sandbox call
  global sigStore, shareddirStore
  shareddirStore=shareddirStore+shareddir
  sigStore['INT']=signal.signal(signal.SIGINT, handler)
  sigStore['TERM']=signal.signal(signal.SIGTERM, handler)
  sigStore['HUP']=signal.signal(signal.SIGHUP, handler)
  sigStore['QUIT']=signal.signal(signal.SIGQUIT, handler)

  ret=1
  try:
    # fix permissions (as the local user)
    if len(shareddir)>0:
      subprocess.check_call([os.path.dirname(__file__)+"/chown_mbsim_mbsimsb", sbuserID]+shareddir)
    # build command for execution on remote side
    # change current dir
    cmdString='cd "'+os.getcwd()+'";'
    # set envars
    for v in envvar:
      if not v in os.environ:
        continue
      cmdString+='export '+v+'="'+os.environ[v]+'";'
    # add user supplied command and save return value
    cmdString+='"'+'" "'.join(cmd)+'"'
    # run command remotely (as the remote user)
    ret=subprocess.call(["ssh", sbuser+"@localhost", cmdString], stdout=stdout, stderr=stderr)
  except:
    print("simplesandbox failed!")
  finally:
    cleanup(shareddir)
    return ret



if __name__=="__main__":
  import argparse
  import sys

  argparser=argparse.ArgumentParser(description="Run command in a simple sandbox")
  argparser.add_argument("cmd", nargs="*", type=str, help="Command and arguments to execute")
  argparser.add_argument("--envvar", nargs="*", type=str, default=[], help="Pass environement variables")
  argparser.add_argument("--shareddir", nargs="*", type=str, default=[], help="Make directories as writable")

  args = argparser.parse_args()

  sys.exit(call(args.cmd, args.envvar, args.shareddir))

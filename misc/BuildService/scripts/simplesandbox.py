#!/usr/bin/python

import os
import subprocess

def call(cmd, envvar=[], shareddir=[], stdout=None, stderr=None):
  sharedGroup="mbsim_mbsimsb" # user this group as shared group (read- and writeable by both)
  user="mbsimsb" # username of the sandboxed user

  # fix permissions (as the local user)
  if len(shareddir)>0:
    for d in shareddir:
      subprocess.call(["sudo", "/home/mbsim/chown_mbsim_mbsimsb.sh", "mbsimsb", os.path.realpath(d)])
  # build command for execution on remote side
  # change current dir
  cmdString="cd "+os.getcwd()+";"
  # set envars
  for v in envvar:
    cmdString+="export "+v+"="+os.environ[v]+";"
  # add user supplied command and save return value
  cmdString+=" ".join(cmd)
  # run command remotely (as the remote user)
  ret=subprocess.call(["ssh", user+"@localhost", cmdString], stdout=stdout, stderr=stderr)
  # fix permissions (as the local user)
  if len(shareddir)>0:
    for d in shareddir:
      subprocess.call(["sudo", "/home/mbsim/chown_mbsim_mbsimsb.sh", "mbsim", os.path.realpath(d)])
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

import subprocess
import sys
import ctypes
import signal
import time
import os.path

p=None

def terminate(a, b):
  if p!=None:
    p.terminate()
    count=0
    while p.poll()==None and count<500:
      time.sleep(0.01)
      count=count+1
    if p.poll()==None:
      p.kill()

signal.signal(signal.SIGUSR1, terminate)

libc=ctypes.CDLL("libc.so.6")
PR_SET_PDEATHSIG=1
if libc.prctl(PR_SET_PDEATHSIG, signal.SIGUSR1, 0, 0, 0)!=0:
  raise auth_plugins.AuthenticationError(log_msg="Cannot call prctl.")

p=subprocess.Popen(['/opt/firejail-local/bin/firejail',
  '--profile=/home/mbsim/SCRIPTS/mbsim/misc/BuildService/scripts/mbsimwebapp-firejail.profile', '--',
  '/usr/bin/python', '/usr/local/mbsim/SCRIPTS/mbsim/misc/BuildService/scripts/mbsimwebapp_service_run.py']+sys.argv[1:])
p.wait()

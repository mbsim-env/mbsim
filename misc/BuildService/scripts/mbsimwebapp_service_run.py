# import standard modules
import sys
import os
import subprocess
import random
import time
import signal
import urlparse
import getpass

def run(token, display):
  # configuration
  # * adaption in /etc/fstab are required
  #   /home/mbsim/linux64-dailydebug /usr/local/mbsim/linux64-dailydebug none bind,ro
  #   /home/mbsim/linux64-dailyrelease /usr/local/mbsim/linux64-dailyrelease none bind,ro
  #   /home/mbsim/win64-dailyrelease /usr/local/mbsim/win64-dailyrelease none bind,ro
  #   /home/mbsim/linux64-ci /usr/local/mbsim/linux64-ci none bind,ro
  #   /home/mbsim/3rdparty /usr/local/mbsim/3rdparty none bind,ro
  #   /home/mbsim/SCRIPTS /usr/local/mbsim/SCRIPTS none bind,ro
  XAUTHTMPL='/tmp/mbsimwebapp-xauth-%d.'+getpass.getuser()
  PIDFILE='/tmp/mbsimwebapp-pid/%d.'+getpass.getuser()

  # check arg
  if type(display)!=int:
    raise RuntimeError('Illegal display parameter')

  # parse the token
  cmd=urlparse.parse_qs(token)

  # get prog of file
  buildType=cmd.get('buildType', [None])[0]
  prog=cmd.get('prog', [None])[0]
  file=cmd.get('file', [])

  # check arg
  if buildType not in ["linux64-dailydebug", "linux64-ci", "linux64-dailyrelease", "win64-dailyrelease"] or \
     prog not in ["openmbv", "h5plotserie", "mbsimgui"]:
    raise RuntimeError('Unknown buildType or prog.')

  # create XAUTH file
  os.open(XAUTHTMPL%(display), os.O_CREAT, 0o600)

  # create XAUTH content
  COOKIE=''.join(random.SystemRandom().choice('0123456789abcdef') for _ in range(32))
  subprocess.check_call(['/usr/bin/xauth', '-f', XAUTHTMPL%(display), 'add', 'localhost:%d'%(display), '.', COOKIE])

  # start Xvnc in background ...
  xvnc=subprocess.Popen(['/opt/tigervnc-local/bin/Xvnc', ':%d'%(display), '-SecurityTypes', 'None',
    '-localhost', '-auth', XAUTHTMPL%(display), "-sigstop", "-NeverShared", "-DisconnectClients"])
  # ... and wait for Xvnc to be ready (Xvnc stops itself, using -sigstop option, if ready; than we continoue it)
  count=0
  while open('/proc/%d/stat'%(xvnc.pid)).readline().split()[2]!='T' and count<1000:
    time.sleep(0.01)
    count=count+1
  if open('/proc/%d/stat'%(xvnc.pid)).readline().split()[2]!='T':
    xvnc.terminate()
    raise RuntimeError("Xvnc failed to start.")
  xvnc.send_signal(signal.SIGCONT) # continue Xvnc
  # create pid file
  open(PIDFILE%(display), 'w').close()

  # prepare env for starting programs in the vnc server
  xenv=os.environ.copy()
  xenv['XAUTHORITY']=XAUTHTMPL%(display)
  xenv['DISPLAY']=':%d'%(display)
  xenv['QT_X11_NO_MITSHM']='1' # required for Qt!??
  xenv['LD_LIBRARY_PATH']='/usr/local/mbsim/3rdparty/casadi3py-local-linux64/lib:/usr/local/mbsim/3rdparty/qwt-6.1.3-local-linux64/lib:/usr/local/mbsim/3rdparty/coin-soqt-bb-local-linux64/lib64'
  xenv['WINEPATH']="/usr/x86_64-w64-mingw32/sys-root/mingw/bin;/usr/local/mbsim/3rdparty/lapack-local-win64/bin;/usr/local/mbsim/3rdparty/xerces-c-local-win64/bin;/usr/local/mbsim/3rdparty/casadi3py-local-win64/lib;/usr/local/mbsim/win64-dailyrelease2/local/bin;/usr/local/mbsim/3rdparty/octave-local-win64/bin;/usr/local/mbsim/3rdparty/hdf5-local-win64/bin;/usr/local/mbsim/3rdparty/libarchive-local-win64/bin;/usr/local/mbsim/3rdparty/python-win64;/usr/local/mbsim/3rdparty/qwt-6.1.3-local-win64/lib;/usr/local/mbsim/3rdparty/coin-soqt-bb-local-win64/bin"

  # run window manager
  wm=subprocess.Popen(['/usr/bin/xfwm4'], env=xenv)

  # run the main program according to token
  absFile=[]
  cdir=None
  for f in file:
    af='/usr/local/mbsim/'+buildType+'/mbsim/examples/'+f
    if os.path.exists(af):
      absFile.append(af)
      if cdir==None: # use first file as current dir for the started program
        cdir=os.path.dirname(af)
  prefixCmd=[]
  if buildType=="win64-dailyrelease":
    # prefix command with wine
    prefixCmd=["/usr/bin/wine"]
    # convert filenames to Windows (wine) path
    for i, v in enumerate(absFile):
      absFile[i] = 'Z:'+v.replace('/', '\\')
  p=subprocess.Popen(prefixCmd+['/usr/local/mbsim/'+buildType+'/local/bin/'+prog, '--fullscreen']+absFile, cwd=cdir, env=xenv)

  # wait for all child processes
  p.wait()
  wm.terminate()
  xvnc.terminate()

if __name__=='__main__':
  run(sys.argv[1], int(sys.argv[2]))

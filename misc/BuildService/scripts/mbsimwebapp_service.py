import sys
sys.path.append('/home/mbsim/Software/websockify')
import websockify.token_plugins
import websockify.auth_plugins
import os.path
import subprocess
import requests
import Cookie
import time
import getpass
import threading

# configuration
DEBUG=True
PIDFILE='/tmp/mbsimwebapp-pid/%d.'+getpass.getuser()

websockify.logger_init()
if DEBUG:
  websockify.logging.getLogger(websockify.WebSocketProxy.log_prefix).setLevel(websockify.logging.DEBUG)

# a global variable to pass over data from token plugin to auth plugin
globalToken=None

# the Websockify token plugin
class MBSimWebappToken(websockify.token_plugins.BasePlugin):
  def lookup(self, token):
    # save the token for later use in the auth module
    global globalToken
    globalToken=token
    # find free display
    def checkDisplayNumber(n):
      # this should be more sophisticated, see file vncserver function CheckDisplayNumber from package tigervnc
      return not os.path.exists('/tmp/.X11-unix/X%d'%(n))
    display=-1
    for i in range(10,31):
      if checkDisplayNumber(i):
        display=i
        break
    if display==-1:
      raise RuntimeError("No free display found")
    # return port of free display
    return ('localhost', 5900+display)

# the Websockify auth plugin
class MBSimWebappAuth(websockify.auth_plugins.BasePlugin):
  def authenticate(self, headers, target_host, target_port):
    # check authentification
    if 'Cookie' not in headers: # error if not Cookie is defined
      raise websockify.auth_plugins.AuthenticationError(log_msg="No cookie provided.")
    # get cookie and get the mbsimenvsessionid form the cookie
    cookie=headers['Cookie']
    c=Cookie.SimpleCookie(cookie)
    if 'mbsimenvsessionid' not in c:
      raise websockify.auth_plugins.AuthenticationError(log_msg="No mbsimenvsessionid provided in cookie.")
    sessionid=c['mbsimenvsessionid'].value
    # call www.mbsim-env.de to check to session ID (we can do this my checking the config file of the server directly
    # but this file is not readable for this user for security reasons)
    response=requests.post('https://www.mbsim-env.de/cgi-bin/mbsimBuildServiceServer.py/checkmbsimenvsessionid',
      json={'mbsimenvsessionid': sessionid})
    # if the response is OK and success is true than continue
    if response.status_code!=200:
      raise websockify.auth_plugins.AuthenticationError(log_msg="Checking session ID failed.")
    d=response.json()
    if 'success' not in d:
      raise websockify.auth_plugins.AuthenticationError(log_msg="Invalid response from mbsim server.")
    if not d['success']:
      raise websockify.auth_plugins.AuthenticationError(log_msg=d['message'])

    token=globalToken
    display=target_port-5900

    # start vnc and other processes in a sandbox
    if os.path.exists(PIDFILE%(display)): os.remove(PIDFILE%(display))
    w=subprocess.Popen(['/usr/bin/python', os.path.dirname(__file__)+'/mbsimwebapp_service_run_wrapper.py', token, str(display)])
    # ... and wait until the pid file exists
    count=0
    while not os.path.exists(PIDFILE%(display)) and count<2000:
      time.sleep(0.01)
      count=count+1
    if not os.path.exists(PIDFILE%(display)):
      w.terminate()
      raise websockify.auth_plugins.AuthenticationError(log_msg="Failed to start vnc server.")

class MyWebSocket(websockify.websockifyserver.CompatibleWebSocket):
  def __init__(self):
    websockify.websockifyserver.CompatibleWebSocket.__init__(self)
    self.lastPing = time.time()                                                                   
    threading.Thread(target=self.checkKill).start()   
  def checkKill(self):                      
    while True:                                 
      time.sleep(5)                             
      if time.time()-self.lastPing>30:            
        websockify.logging.getLogger(websockify.WebSocketProxy.log_prefix).info('Client seems to be dead. No ping since 30sec. Killing now.')
        os._exit(0)        
  def handle_pong(self, data):                                                                        
    self.lastPing = time.time()                 

class MyPRH(websockify.websocketproxy.ProxyRequestHandler):
  SocketClass=MyWebSocket

server = websockify.websocketproxy.WebSocketProxy(
  RequestHandlerClass=MyPRH,
  listen_host='localhost',
  listen_port=10080,
  heartbeat=10,
  token_plugin=MBSimWebappToken(None),
  auth_plugin=MBSimWebappAuth(None),
)
server.start_server()

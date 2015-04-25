#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2

try:

  # imports
  import os
  import urlparse
  import requests
  import json
  import hmac
  import hashlib
  import sys
  import time
  import threading
  import fcntl

  # config file: this will lock the config file
  class ConfigFile:
    def __init__(self, rw):
      self.rw=rw
    def __enter__(self):
      configFilename="/home/user/Tools/runexamples-refupdate-cgi.py.config"
      if self.rw:
        self.fd=open(configFilename, 'r+')
        fcntl.lockf(self.fd, fcntl.LOCK_EX)
        self.config=json.load(self.fd)
      else:
        fd=open(configFilename, 'r')
        fcntl.lockf(fd, fcntl.LOCK_SH)
        self.config=json.load(fd)
        fcntl.lockf(fd, fcntl.LOCK_UN)
        fd.close()
      return self.config
    def __exit__(self, exc_type, exc_value, traceback):
      if self.rw:
        self.fd.seek(0);
        json.dump(self.config, self.fd)
        self.fd.truncate();
        fcntl.lockf(self.fd, fcntl.LOCK_UN)
        self.fd.close()
  
  # default response
  defaultOutput=True
  response_data={'success': False, 'message': "Internal error: Unknown action."}
  
  # get the script action = path info after the script url
  action=os.environ.get('PATH_INFO', None)

  def checkCredicals(config):
    # get login and athmac by http get methode
    data=json.load(sys.stdin)
    login=data.get('login', None)
    if login==None:
      response_data['success']=False
      response_data['message']="Not logged in. Please login before saving."
    else:
      athmac=data['athmac']
      # check whether login in already known by the server (logged in)
      if login not in config['login_access_token']:
        response_data['success']=False
        response_data['message']="User "+login+" not known on the server. Please login before saving."
      else:
        # get access token for login
        access_token=config['login_access_token'][login]
        # check whether the athmac is correct
        if hmac.new(config['client_secret'].encode('utf-8'), access_token, hashlib.sha1).hexdigest()!=athmac:
          response_data['success']=False
          response_data['message']="Invalid access token hmac! Maybe the login was faked! If not, try to relogin again."
        else:
          # check whether this login is permitted to save data on the server (query github collaborators)
          headers={'Authorization': 'token '+access_token}
          response=requests.get('https://api.github.com/teams/1451964/memberships/%s'%(login), headers=headers)
          if response.status_code!=200:
            response_data['success']=False
            response_data['message']="Not allowed to save, since you ("+login+") are not a member of the team Developers of the organization mbsim-env. Please login as a user with valid permissions."
          elif response.json()['state']!='active':
            response_data['success']=False
            response_data['message']="Not allowed to save, since your ("+login+") status in the team Developers of the organization mbsim-env is pending."
          else:
            response_data['success']=True
    return data, response_data

  # generate a html page which stores the login and access token hmac in the browser
  if action=="/saveLoginInBrowser":
    defaultOutput=False
    print('Content-Type: text/html')
    print()
    print('''<!DOCTYPE html>
<html lang="en">
  <head>
    <META http-equiv="Content-Type" content="text/html; charset=UTF-8">
    <title>Save Login in Browser</title>
  </head>
  <body style="margin:1em">
    <script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>
    <script type="text/javascript">
      $(document).ready(function() {
        // helper function: return the query string as a json object
        jQuery.extend({
          getQueryParameters : function(str) {
            return (str || document.location.search).replace(/(^\?)/,'').split("&").map(function(n) {
              return n = n.split("="),this[n[0]] = n[1],this
            }.bind({}))[0];
          }
        });
        // handle GET string (from a login redirect)
        var query=$.getQueryParameters();
        if('login' in query) {
          // save login and access token hmac
          localStorage["GITHUB_LOGIN_NAME"]=query.login;
          localStorage["GITHUB_LOGIN_ATHMAC"]=query.athmac;
          // notify opener window
          window.opener.postMessage("User "+query.login+" successfully logged in.", window.location);
        }
      })
    </script>
    <h1>Please Wait</h1>
    <p>Saving login name and hmac of access token in browser.</p>
    <p>This window should close itself after a short time, if not close it manually.</p>
  </body>
</html>''')
  
  # login using github
  if action=="/login":
    # get the github code passed provided by html get methode
    query=urlparse.parse_qs(os.environ['QUERY_STRING'])
    if 'error' in query:
      response_data['success']=False
      response_data['message']="Authorization request failed: "+query['error']
    else:
      code=query['code']
      with ConfigFile(True) as config:
        # get access token from github (as a json response)
        data={'client_id': '2d12f6576e23c7ba04a4', 'client_secret': config['client_secret'], 'code': code}
        headers={'Accept': 'application/json'}
        response=requests.post('https://github.com/login/oauth/access_token', headers=headers, data=data).json()
        if 'error' in response:
          response_data['success']=False
          response_data['message']="Access token request failed: "+response['error']
        else:
          access_token=response['access_token']
          # get github login name using github API request
          headers={'Authorization': 'token '+access_token}
          response=requests.get('https://api.github.com/user', headers=headers).json()
          login=response['login']
          # save login and access token in a dictionary on the server
          config['login_access_token'][login]=access_token
          # redirect to the example web side and pass login and access token hmac as http get methode
          athmac=hmac.new(config['client_secret'].encode('utf-8'), access_token, hashlib.sha1).hexdigest()
          defaultOutput=False
          print('Location: http://%s%s/saveLoginInBrowser?login=%s&athmac=%s'%(os.environ['HTTP_HOST'], os.environ['SCRIPT_NAME'], login, athmac))
          print()

  # logout
  if action=="/logout":
    # get login which should be logged by html get methode
    data=json.load(sys.stdin)
    login=data.get('login', None)
    if login==None:
      response_data['success']=True
      response_data['message']="Not logged in."
    else:
      with ConfigFile(True) as config:
        # remove login including access_token from server config
        config['login_access_token'].pop(login, None)
        # generate json response
        response_data['success']=True
        response_data['message']="Logged "+login+" out from browser and server."
  
  # return current checked examples
  if action=="/getcheck":
    with ConfigFile(False) as config: pass
    # not json input via http post required
    # return the checkedExamples entries of the config as json response
    response_data['success']=True
    response_data['message']="To be updated examples loaded."
    response_data['checkedExamples']=config['checkedExamples']
  
  # save checked examples (if logged in)
  if action=="/setcheck":
    with ConfigFile(True) as config:
      data, response_data=checkCredicals(config)
      if response_data['success']:
        # save checked examples
        config['checkedExamples']=data['checkedExamples']
        # response
        response_data['success']=True
        response_data['message']="Successfully saved."

  # return current checked examples
  if action=="/getcibranches":
    with ConfigFile(False) as config: pass
    # not json input via http post required
    # return branches for CI
    data=json.load(sys.stdin)
    login=data.get('login', None)
    if login==None:
      curcibranch=config['curcibranch']
      response_data['success']=True
      response_data['message']="Continuous integration braches loaded."
      response_data['curcibranch']=curcibranch
      response_data['fmatvecbranch']=[]
      response_data['hdf5seriebranch']=[]
      response_data['openmbvbranch']=[]
      response_data['mbsimbranch']=[]
    else:
      access_token=config['login_access_token'][login]
      headers={'Authorization': 'token '+access_token}
      # worker function to make github api requests in parallel
      def getBranch(url, headers, out):
        out.extend([b['name'] for b in requests.get(url, headers=headers).json()])
      # output data placeholder, request url and thread object placeholder. all per thread (reponame)
      out={'fmatvec': [], 'hdf5serie': [], 'openmbv': [], 'mbsim': []}
      url={'fmatvec': 'https://api.github.com/repos/friedrichatgc/casadi/branches',
           'hdf5serie': 'https://api.github.com/repos/friedrichatgc/casadi/branches',
           'openmbv': 'https://api.github.com/repos/friedrichatgc/casadi/branches',
           'mbsim': 'https://api.github.com/repos/friedrichatgc/casadi/branches'}
      thread={}
      # make all calls in parallel
      for reponame in out:
        thread[reponame]=threading.Thread(target=getBranch, args=(url[reponame], headers, out[reponame]))
        thread[reponame].start()
      # wait for all calls
      for reponame in out:
        thread[reponame].join(10)
      curcibranch=config['curcibranch']
      response_data['success']=True
      response_data['message']="Continuous integration braches loaded."
      response_data['curcibranch']=curcibranch
      response_data['fmatvecbranch']=out['fmatvec']
      response_data['hdf5seriebranch']=out['hdf5serie']
      response_data['openmbvbranch']=out['openmbv']
      response_data['mbsimbranch']=out['mbsim']

  # return current checked examples
  if action=="/addcibranch":
    with ConfigFile(True) as config:
      data, response_data=checkCredicals(config)
      if response_data['success']:
        # save checked examples
        newcibranch=data['addcibranch']
        curcibranch=config['curcibranch']
        add=True
        for c in curcibranch:
          if c['fmatvec']==newcibranch['fmatvec'] and c['hdf5serie']==newcibranch['hdf5serie'] and \
             c['openmbv']==newcibranch['openmbv'] and c['mbsim']==newcibranch['mbsim']:
            add=False
            break
        if add:
          curcibranch.append(newcibranch);
        response_data['message']='New CI branch combination saved.'

  # return current checked examples
  if action=="/delcibranch":
    with ConfigFile(True) as config:
      data, response_data=checkCredicals(config)
      if response_data['success']:
        # del ci branch
        delcibranch=data['delcibranch']
        newcurcibranch=[]
        for c in config['curcibranch']:
          # never delete master
          if c['fmatvec']=='master' and c['hdf5serie']=='master' and \
             c['openmbv']=='master' and c['mbsim']=='master':
            newcurcibranch.append(c)
          else:
            # do not add the deleted one
            if c['fmatvec']==delcibranch['fmatvec'] and c['hdf5serie']==delcibranch['hdf5serie'] and \
               c['openmbv']==delcibranch['openmbv'] and c['mbsim']==delcibranch['mbsim']:
              continue
            # add others
            newcurcibranch.append(c)
        config['curcibranch']=newcurcibranch
        response_data['message']='CI branch combination deleted.'

  # get user information
  if action=="/getuser":
    # not json input via http post required
    # return branches for CI
    data=json.load(sys.stdin)
    login=data.get('login', None)
    if login==None:
      response_data['success']=False
      response_data['message']="Not logged in."
    else:
      with ConfigFile(False) as config: pass
      access_token=config['login_access_token'][login]
      headers={'Authorization': 'token '+access_token}
      response=requests.get('https://api.github.com/user', headers=headers).json()
      response_data['success']=True
      response_data['message']="User information returned."
      response_data['username']=response['name']+" ("+login+")"
      response_data['avatarurl']=response['avatar_url']

  # react on web hooks
  if action=="/webhook":
    with ConfigFile(True) as config:
      rawdata=sys.stdin.read()
      sig=os.environ['HTTP_X_HUB_SIGNATURE'][5:]
      if sig!=hmac.new(config['webhook_secret'].encode('utf-8'), rawdata, hashlib.sha1).hexdigest():
        response_data['success']=False
        response_data['message']="Invalid signature. Only github is allowed to send hooks."
      else:
        data=json.loads(rawdata)
        # get current config
        curcibranch=config['curcibranch']
        tobuild=config['tobuild']
        # get repo and branch from this push
        repo=data['repository']['name']
        branch=data['ref'][11:]
        # update tobuild
        for c in curcibranch:
          if c[repo]==branch:
            toadd=c
            toadd['timestamp']=int(time.time())
            tobuild.append(toadd)
        # create response
        response_data['success']=True
        response_data['message']="OK"


except:
  # reset all output and generate a json error message
  import traceback
  defaultOutput=True
  response_data={}
  response_data['success']=False
  response_data['message']="Internal error: Please report the following error to the maintainer:\n"+traceback.format_exc()
  
# generate response
if defaultOutput:
  print('Content-Type: application/json')
  print()
  print(json.dumps(response_data))

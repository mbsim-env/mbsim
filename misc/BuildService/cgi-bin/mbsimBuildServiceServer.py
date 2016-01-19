#!/usr/bin/python

from __future__ import print_function # to enable the print function for backward compatiblity with python2
import json

try:

  # imports
  import os
  import urlparse
  import requests
  import hmac
  import hashlib
  import sys
  import time
  import threading
  import fcntl
  import datetime
  import Cookie
  import re
  import shutil

  # config file: this will lock the config file for the lifetime of this object
  class ConfigFile:
    def __init__(self, rw):
      self.rw=rw
    def __enter__(self):
      configFilename="/home/mbsim/BuildServiceConfig/mbsimBuildService.conf"
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
        self.fd.seek(0)
        json.dump(self.config, self.fd)
        self.fd.truncate()
        fcntl.lockf(self.fd, fcntl.LOCK_UN)
        self.fd.close()
  
  # get the script action = path info after the script url
  action=os.environ.get('PATH_INFO', None)
  method=os.environ.get('REQUEST_METHOD', None)
  
  # default response
  defaultOutput=True
  response_data={'success': False, 'message': "Internal error: Unknown action or request method: "+action}

  def checkCredicals(config):
    # get login and athmac by http get methode
    if 'HTTP_COOKIE' in os.environ:
      c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
      login=c['mbsimenvsessionuser'].value
      athmac=c['mbsimenvsessionid'].value
    else:
      login=None
      athmac=None
    # check
    if login==None or athmac==None:
      response_data['success']=False
      response_data['message']="Not logged in. Please login before saving."
    else:
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
          headers={'Authorization': 'token '+access_token,
                   'Accept': 'application/vnd.github.v3+json'}
          response=requests.get('https://api.github.com/teams/1451964/memberships/%s'%(login), headers=headers)
          if response.status_code!=200:
            response_data['success']=False
            response_data['message']="Not allowed to save, since you ("+login+") are not a member of the team Developers of the organization mbsim-env. Please login as a user with valid permissions."
          elif response.json()['state']!='active':
            response_data['success']=False
            response_data['message']="Not allowed to save, since your ("+login+") status in the team Developers of the organization mbsim-env is pending."
          else:
            response_data['success']=True
    return response_data
  
  # login using github
  if action=="/login" and method=="GET":
    # get the github code passed provided by html get methode
    query=urlparse.parse_qs(os.environ['QUERY_STRING'])
    if 'error' in query:
      response_data['success']=False
      response_data['message']="Authorization request failed: "+query['error']
    else:
      code=query['code']
      with ConfigFile(True) as config:
        # get access token from github (as a json response)
        data={'client_id': '987997eb60fc086e9707', 'client_secret': config['client_secret'], 'code': code}
        headers={'Accept': 'application/json'}
        response=requests.post('https://github.com/login/oauth/access_token', headers=headers, data=data).json()
        if 'error' in response:
          response_data['success']=False
          response_data['message']="Access token request failed: "+response['error']
        else:
          access_token=response['access_token']
          # get github login name using github API request
          headers={'Authorization': 'token '+access_token,
                   'Accept': 'application/vnd.github.v3+json'}
          response=requests.get('https://api.github.com/user', headers=headers).json()
          login=response['login']
          # save login and access token in a dictionary on the server
          config['login_access_token'][login]=access_token
          # redirect to the example web side and pass login and access token hmac as http get methode
          athmac=hmac.new(config['client_secret'].encode('utf-8'), access_token, hashlib.sha1).hexdigest()
          # create cookie
          c=Cookie.SimpleCookie()
          c['mbsimenvsessionuser']=login
          c['mbsimenvsessionuser']['comment']="Session username of the mbsimenvsessionid cookie"
          c['mbsimenvsessionuser']['domain']='.www.ssl-id1.de'
          c['mbsimenvsessionuser']['path']='/mbsim-env.de'
          c['mbsimenvsessionuser']['secure']=True
          c['mbsimenvsessionuser']['httponly']=True
          c['mbsimenvsessionid']=athmac
          c['mbsimenvsessionid']['comment']="Session ID for www.mbsim-env.de"
          c['mbsimenvsessionid']['domain']='.www.ssl-id1.de'
          c['mbsimenvsessionid']['path']='/mbsim-env.de'
          c['mbsimenvsessionid']['secure']=True
          c['mbsimenvsessionid']['httponly']=True
          defaultOutput=False
          print('Content-Type: text/html')
          print(c)
          print()
          print('''<!DOCTYPE html>
<html lang="en">
  <head>
    <META http-equiv="Content-Type" content="text/html; charset=UTF-8">
    <title>Set cookie and notify opener windows</title>
    <link rel="shortcut icon" href="data:image/x-icon;," type="image/x-icon"/>
  </head>
  <body style="margin:1em">
    <script type="text/javascript" src="https://code.jquery.com/jquery-2.1.4.min.js"> </script>
    <script type="text/javascript">
      $(document).ready(function() {
        // notify opener window
        window.opener.postMessage("User %s successfully logged in.", "http://www.mbsim-env.de");
        window.opener.postMessage("User %s successfully logged in.", "https://www.ssl-id1.de");
      })
    </script>
    <h1>Please Wait</h1>
    <p>Set cookie in your browser.</p>
    <p>This window should close itself after a short time.</p>
  </body>
</html>'''%(login, login))

  # logout
  if action=="/logout" and method=="GET":
    # get login
    if 'HTTP_COOKIE' in os.environ:
      c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
      login=c['mbsimenvsessionuser'].value
    else:
      login=None
    if login==None:
      response_data['success']=True
      response_data['message']="Nobody to log out."
    else:
      with ConfigFile(True) as config:
        # remove login including access_token from server config
        config['login_access_token'].pop(login, None)
        # generate json response
        response_data['success']=True
        response_data['message']="Logged "+login+" out from server."
  
  # return current checked examples
  if action=="/getcheck" and method=="GET":
    with ConfigFile(False) as config: pass
    # not json input via http post required
    # return the checkedExamples entries of the config as json response
    response_data['success']=True
    response_data['message']="To be updated examples loaded."
    response_data['checkedExamples']=config['checkedExamples']
  
  # save checked examples (if logged in)
  if action=="/setcheck" and method=="POST":
    with ConfigFile(True) as config:
      response_data=checkCredicals(config)
      data=json.load(sys.stdin)
      if response_data['success']:
        # save checked examples
        config['checkedExamples']=data['checkedExamples']
        # response
        response_data['success']=True
        response_data['message']="Successfully saved."

  # return current checked examples
  if action=="/getcibranches" and method=="GET":
    with ConfigFile(False) as config: pass
    # no json input via http post required
    # return branches for CI
    # worker function to make github api requests in parallel
    def getBranch(url, headers, out):
      out.extend([b['name'] for b in requests.get(url, headers=headers).json()])
    # output data placeholder, request url and thread object placeholder. all per thread (reponame)
    out={'fmatvec': [], 'hdf5serie': [], 'openmbv': [], 'mbsim': []}
    url={'fmatvec': 'https://api.github.com/repos/mbsim-env/fmatvec/branches',
         'hdf5serie': 'https://api.github.com/repos/mbsim-env/hdf5serie/branches',
         'openmbv': 'https://api.github.com/repos/mbsim-env/openmbv/branches',
         'mbsim': 'https://api.github.com/repos/mbsim-env/mbsim/branches'}
    thread={}
    # make all calls in parallel
    headers={'Accept': 'application/vnd.github.v3+json'}
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
  if action=="/addcibranch" and method=="POST":
    with ConfigFile(True) as config:
      response_data=checkCredicals(config)
      data=json.load(sys.stdin)
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
          curcibranch.append(newcibranch)
        response_data['message']='New CI branch combination saved.'

  # return current checked examples
  if action=="/delcibranch" and method=="POST":
    with ConfigFile(True) as config:
      response_data=checkCredicals(config)
      data=json.load(sys.stdin)
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
  if action=="/getuser" and method=="GET":
    if 'HTTP_COOKIE' in os.environ:
      c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
      login=c['mbsimenvsessionuser'].value
    else:
      login=None
    if login==None:
      response_data['success']=True
      response_data['username']="Not logged in"
      response_data['message']="No session ID cookie found on your browser."
    else:
      with ConfigFile(False) as config: pass
      if not login in config['login_access_token']:
        response_data['success']=True
        response_data['username']="Not logged in"
        response_data['message']="The username of the browser cookie is not known by the server. Please relogin."
      else:
        response_data['success']=True
        response_data['username']=login
        response_data['message']="User information returned."

  # react on web hooks
  if action=="/webhook" and method=="POST":
    with ConfigFile(True) as config:
      rawdata=sys.stdin.read()
      sig=os.environ['HTTP_X_HUB_SIGNATURE'][5:]
      if sig!=hmac.new(config['webhook_secret'].encode('utf-8'), rawdata, hashlib.sha1).hexdigest():
        response_data['success']=False
        response_data['message']="Invalid signature. Only github is allowed to send hooks."
      else:
        response_data['success']=True
        response_data['message']="not implemented yet"
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
            toadd=c.copy()
            toadd['timestamp']=int(time.time())
            tobuild.append(toadd)
        # create response
        response_data['success']=True
        response_data['message']="OK"

  # copy distribution to release and tag on github
  if action=="/releasedistribution" and method=="POST":
    with ConfigFile(False) as config: pass
    response_data=checkCredicals(config)
    data=json.load(sys.stdin)
    if response_data['success']:
      data['relStr']=data['relStr']+"-TESTING_DO_NOT_USE"#mfmf testing: delete this line
      # generate tagname, platform and relArchiveName
      tagName=re.sub("mbsim-env-(.*)-shared-build-xxx\..*", "release/"+data['relStr']+"-\\1", data['distArchiveName'])
      platform=re.sub("mbsim-env-(.*)-shared-build-xxx\..*", "\\1", data['distArchiveName'])
      relArchiveName=re.sub("(mbsim-env-.*-shared-build-)xxx(\..*)", "\\g<1>"+data['relStr']+"\\2", data['distArchiveName'])
      # access token from config file and standard http header
      access_token=config['login_access_token'][data['login']]
      headers={'Authorization': 'token '+access_token,
               'Accept': 'application/vnd.github.v3+json'}
      # the default response -> is changed/appended later
      response_data['success']=True
      response_data['message']=""
      # the current time -> the create date of the tag object
      curtime=datetime.datetime.utcnow()
      # get user name -> is used in the tag object
      response=requests.get('https://api.github.com/user', headers=headers).json()
      name=response['name']
      email=response['email']
      # create tag object and reference for all repositories
      data['commitid']['mdtest']='de85f2642ec61d6a62fe87d3e3d2cef18669d620'#mfmf testing: delete this line
      org="friedrichatgc"# mfmf testing: delete this line
      repos=['mdtest']# mfmf testing: delete this lin
      #mfmf org="mbsim-env"
      #mfmf repos=['fmatvec', 'hdf5serie', 'openmbv', 'mbsim']
      for repo in repos:
        # create the git tag object
        createTagData={"tag": tagName,
                       "message": "Release "+data['relStr']+" of MBSim-Env for "+platform+"\n"+\
                                  "\n"+\
                                  "The binary "+platform+" release can be downloaded from\n"+\
                                  "http://www.mbsim-env.de/mbsim/releases/"+relArchiveName+"\n"+\
                                  "Please note that this binary release includes a full build of MBSim-Env not only of this repository.\n"+\
                                  "Also look at http://www.mbsim-env.de/mbsim/releases for other platforms of this release version.\n",
                       "object": data['commitid'][repo],
                       "type": "commit",
                       "tagger": {
                         "name": name,
                         "email": name,
                         "date": curtime.strftime("%Y-%m-%dT%H:%M:%SZ")
                      }}
        response=requests.post('https://api.github.com/repos/'+org+'/'+repo+'/git/tags',
                               headers=headers, data=json.dumps(createTagData))
        # check if the create was successfull
        if response.status_code!=201:
          # not successfull -> set response_data and continue with next repo
          response_data['success']=False
          response_data['message']=response_data['message']+" "+\
            "Unable to create the git tag object for repo "+repo+". Please check the tag status of (at least) this repository manually."
        else:
          # git tag object created -> get the tag object id
          tagObjectID=response.json()['sha']
          # create the github ref
          createRefData={"ref": "refs/tags/"+tagName,
                         "sha": tagObjectID}
          response=requests.post('https://api.github.com/repos/'+org+'/'+repo+'/git/refs',
                                 headers=headers, data=json.dumps(createRefData))
          # check if the create was successfull
          if response.status_code!=201:
            # not successfull -> set response_data and continue with next repo
            response_data['success']=False
            response_data['message']=response_data['message']+" "+\
              "Unable to create the git reference for repo "+repo+". Please check the tag status of (at least) this repository manually."
          else:
            # git ref object created -> create GitHub release info
            createRelData={"tag_name": tagName,
                           "name": "Release "+data['relStr']+" of MBSim-Env for "+platform,
                           "body": "The binary "+platform+" release can be downloaded from\n"+\
                                   "http://www.mbsim-env.de/mbsim/releases/"+relArchiveName+"\n"+\
                                   "Please note that this binary release includes a full build of MBSim-Env not only of this repository. "+\
                                   "Also look at http://www.mbsim-env.de/mbsim/releases for other platforms of this release version.",
                           "draft": False,
                           "prerelease": False}
            response=requests.post('https://api.github.com/repos/'+org+'/'+repo+'/releases',
                                   headers=headers, data=json.dumps(createRelData))
            # check if the update was successfull
            if response.status_code!=201:
              # not successfull -> set response_data and continue with next repo
              response_data['success']=False
              response_data['message']=response_data['message']+" "+\
                "Unable to create the GitHub release info for repo "+repo+". Please check the tag/release status of (at least) this repository manually."
      # set message if everything was done Successfully
      if response_data['success']==True:
        response_data['message']="Successfully released and tagged this distribution."
        # copy the distribution to the release dir
        srcFile=data['reportOutDir']+"/distribute/"+data['distArchiveName']
        dstFile="/var/www/html/mbsim/releases/"+relArchiveName
        shutil.copyfile(srcFile, dstFile)

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
  print('Access-Control-Allow-Origin: http://www.mbsim-env.de') # allow CORS from www.mbsim-env.de
  print('Access-Control-Allow-Credentials: true')
  print()
  print(json.dumps(response_data))

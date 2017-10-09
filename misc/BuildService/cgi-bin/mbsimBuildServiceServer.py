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
  class ConfigFile(object):
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
        json.dump(self.config, self.fd, indent=2)
        self.fd.truncate()
        fcntl.lockf(self.fd, fcntl.LOCK_UN)
        self.fd.close()

  def checkCredicals(config, sessionid=None):
    if sessionid==None and 'HTTP_COOKIE' in os.environ:
      cookie=os.environ["HTTP_COOKIE"]
      c=Cookie.SimpleCookie(cookie)
      sessionid=c['mbsimenvsessionid'].value
    # check
    response_data={'success': False, 'message': 'Unknown'}
    if sessionid==None:
      response_data['success']=False
      response_data['message']="Not logged in."
    else:
      # check whether sessionid is known by the server (logged in)
      if sessionid not in config['session']:
        response_data['success']=False
        response_data['message']="Unknown session ID."
      else:
        # get access token for login
        access_token=config['session'][sessionid]['access_token']
        # check whether the sessionid is correct
        if not hmac.compare_digest(hmac.new(config['client_secret'].encode('utf-8'), access_token, hashlib.sha1).hexdigest(), sessionid.encode('utf-8')):
          response_data['success']=False
          response_data['message']="Invalid access token hmac! Maybe the login was faked! If not, try to relogin again."
        else:
          # check whether this login is permitted to save data on the server (query github collaborators)
          headers={'Authorization': 'token '+access_token,
                   'Accept': 'application/vnd.github.v3+json'}
          login=config['session'][sessionid]['login']
          response=requests.get('https://api.github.com/teams/1451964/memberships/%s'%(login), headers=headers)
          if response.status_code!=200:
            response_data['success']=False
            response_data['message']="Not allowed, since you ("+login+") are not a member of the team Developers of the organization mbsim-env. Please login as a user with valid permissions."
          elif response.json()['state']!='active':
            response_data['success']=False
            response_data['message']="Not allowed, since your ("+login+") status in the team Developers of the organization mbsim-env is pending."
          else:
            response_data['success']=True
    return response_data

  # remove all sessionid's with username login from config
  def removeLogin(config, login):
    so=config['session']
    sn={}
    for s in so:
      if so[s]['login']!=login:
        sn[s]=so[s]
    config['session']=sn

  if __name__ == "__main__":
    # get the script action = path info after the script url
    action=os.environ.get('PATH_INFO', None)
    method=os.environ.get('REQUEST_METHOD', None)
    
    # default response
    defaultOutput=True
    response_data={'success': False, 'message': "Internal error: Unknown action or request method: "+action}
    
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
            response=requests.get('https://api.github.com/user', headers=headers)
            if response.status_code!=200:
              response_data['success']=False
              response_data['message']="Internal error. Cannot get user information."
            else:
              response=response.json()
              login=response['login']
              # redirect to the example web side and pass login and access token hmac as http get methode
              sessionid=hmac.new(config['client_secret'].encode('utf-8'), access_token, hashlib.sha1).hexdigest()
              # save login and access token in a dictionary on the server (first remove all sessionid for login than add new sessionid)
              removeLogin(config, login)
              config['session'][sessionid]={'access_token': access_token,
                                            'login': login,
                                            'avatar_url': response['avatar_url'],
                                            'name': response['name']}
              # create cookie
              c=Cookie.SimpleCookie()
              # sessionid cookie not visible to javascript
              c['mbsimenvsessionid']=sessionid
              c['mbsimenvsessionid']['comment']="Session ID for www.mbsim-env.de"
              c['mbsimenvsessionid']['domain']='www.mbsim-env.de'
              c['mbsimenvsessionid']['path']='/'
              c['mbsimenvsessionid']['secure']=True
              c['mbsimenvsessionid']['httponly']=True
              # javascript cookie just to show the the user is logged in
              c['mbsimenvsessionid_js']="logged_in"
              c['mbsimenvsessionid_js']['comment']="User logged into for www.mbsim-env.de"
              c['mbsimenvsessionid_js']['domain']='www.mbsim-env.de'
              c['mbsimenvsessionid_js']['path']='/'
              c['mbsimenvsessionid_js']['secure']=True
              defaultOutput=False
              print('Status: 301 Moved Permanently')
              print('Location: https://www.mbsim-env.de/mbsim/html/index.html')
              print(c)
              print()

    # logout
    if action=="/logout" and method=="GET":
      # get login
      if 'HTTP_COOKIE' in os.environ:
        c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
        sessionid=c['mbsimenvsessionid'].value
      else:
        sessionid=None
      if sessionid==None:
        response_data['success']=True
        response_data['message']="Nobody to log out."
      else:
        with ConfigFile(True) as config:
          # remove all sessionids for login from server config
          removeLogin(config, config['session'][sessionid]['login'])
          # generate json response
          response_data['success']=True
          response_data['message']="Logged out from server."
    
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

    # return current ci branches of all repos
    if action=="/getcibranches" and method=="GET":
      with ConfigFile(False) as config: pass
      repos=['fmatvec', 'hdf5serie', 'openmbv', 'mbsim']
      # use a authentificated request if logged in (to avoid rate limit problems on github)
      headers={'Accept': 'application/vnd.github.v3+json'}
      if 'HTTP_COOKIE' in os.environ:
        c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
        sessionid=c['mbsimenvsessionid'].value
        if sessionid in config['session']:
          headers['Authorization']='token '+config['session'][sessionid]['access_token']
      # worker function to make github api requests in parallel
      def getBranch(repo, out):
        response=requests.get('https://api.github.com/repos/mbsim-env/'+repo+'/branches', headers=headers)
        if response.status_code==200:
          out.extend([b['name'] for b in response.json()])
      # start worker threads
      thread={}
      out={}
      for repo in repos:
        out[repo]=[]
        thread[repo]=threading.Thread(target=getBranch, args=(repo, out[repo]))
        thread[repo].start()
      # wait for all threads
      for repo in repos:
        thread[repo].join()
      # generate output
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
          # update tobuild
          tobuild=config['tobuild']
          toadd=newcibranch.copy()
          toadd['timestamp']=int(time.time())
          tobuild.append(toadd)
          # response
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
        sessionid=c['mbsimenvsessionid'].value
      else:
        sessionid=None
      if sessionid==None:
        response_data['success']=True
        response_data['username']=None
        response_data['avatar_url']=''
        response_data['message']="No session ID cookie found on your browser."
      else:
        with ConfigFile(False) as config: pass
        if not sessionid in config['session']:
          response_data['success']=True
          response_data['username']=None
          response_data['avatar_url']=''
          response_data['message']="The username of the browser cookie is not known by the server. Please relogin."
        else:
          response_data['success']=True
          response_data['username']=config['session'][sessionid]['name']+" ("+config['session'][sessionid]['login']+")"
          response_data['avatar_url']=config['session'][sessionid]['avatar_url']
          response_data['message']="User information returned."

    # react on web hooks
    if action=="/webhook" and method=="POST":
      with ConfigFile(True) as config:
        rawdata=sys.stdin.read()
        sig=os.environ['HTTP_X_HUB_SIGNATURE'][5:]
        if not hmac.compare_digest(sig, hmac.new(config['webhook_secret'].encode('utf-8'), rawdata, hashlib.sha1).hexdigest()):
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
        # generate tagname, platform and relArchiveName
        tagName=re.sub("mbsim-env-(.*)-shared-build-xxx\..*", "release/"+data['relStr']+"-\\1", data['distArchiveName'])
        platform=re.sub("mbsim-env-(.*)-shared-build-xxx\..*", "\\1", data['distArchiveName'])
        relArchiveName=re.sub("mbsim-env-.*-shared-build-xxx(\..*)", "mbsim-env-release-"+data['relStr']+"-"+platform+"\\1", data['distArchiveName'])
        # access token from config file and standard http header
        c=Cookie.SimpleCookie(os.environ["HTTP_COOKIE"])
        sessionid=c['mbsimenvsessionid'].value
        access_token=config['session'][sessionid]['access_token']
        headers={'Authorization': 'token '+access_token,
                 'Accept': 'application/vnd.github.v3+json'}
        # the default response -> is changed/appended later
        response_data['success']=True
        response_data['message']=""
        # the current time -> the create date of the tag object
        curtime=datetime.datetime.utcnow()
        # get user email
        response=requests.get('https://api.github.com/user/emails', headers=headers)
        if response.status_code!=200:
          response_data['success']=False
          response_data['message']="Internal error. Cannot get email address of user."
        else:
          response=response.json()
          # get primary email
          for item in response:
            if item['primary']:
              email=item['email']
          # create tag object, create git reference and create a github release for all repositories
          org="mbsim-env"
          repos=['fmatvec', 'hdf5serie', 'openmbv', 'mbsim']
          # worker function to make github api requests in parallel
          def tagRefRelease(repo, out):
            # create the git tag object
            createTagData={"tag": tagName,
                           "message": "Release "+data['relStr']+" of MBSim-Env for "+platform+"\n"+\
                                      "\n"+\
                                      "The binary "+platform+" release can be downloaded from\n"+\
                                      "https://www.mbsim-env.de/mbsim/releases/"+relArchiveName+"\n"+\
                                      "Please note that this binary release includes a full build of MBSim-Env not only of this repository.\n"+\
                                      "Also look at https://www.mbsim-env.de/mbsim/releases for other platforms of this release version.\n",
                           "object": data['commitid'][repo],
                           "type": "commit",
                           "tagger": {
                             "name": config['session'][sessionid]['name'],
                             "email": email,
                             "date": curtime.strftime("%Y-%m-%dT%H:%M:%SZ")
                          }}
            response=requests.post('https://api.github.com/repos/'+org+'/'+repo+'/git/tags',
                                   headers=headers, data=json.dumps(createTagData))
            # check if the create was successfull
            if response.status_code!=201:
              # not successfull -> set out
              out['success']=False
              out['message']="Unable to create the git tag object for repo "+repo+". Please check the tag status of (at least) this repository manually; "
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
                # not successfull -> set out
                out['success']=False
                out['message']="Unable to create the git reference for repo "+repo+". Please check the tag status of (at least) this repository manually; "
              else:
                # git ref object created -> create GitHub release info
                createRelData={"tag_name": tagName,
                               "name": "Release "+data['relStr']+" of MBSim-Env for "+platform,
                               "body": "The binary "+platform+" release can be downloaded from\n"+\
                                       "https://www.mbsim-env.de/mbsim/releases/"+relArchiveName+"\n"+\
                                       "Please note that this binary release includes a full build of MBSim-Env not only of this repository. "+\
                                       "Also look at https://www.mbsim-env.de/mbsim/releases for other platforms of this release version.",
                               "draft": False,
                               "prerelease": False}
                response=requests.post('https://api.github.com/repos/'+org+'/'+repo+'/releases',
                                       headers=headers, data=json.dumps(createRelData))
                # check if the update was successfull
                if response.status_code!=201:
                  # not successfull -> set out
                  out['success']=False
                  out['message']="Unable to create the GitHub release info for repo "+repo+". "+\
                                 "Please check the tag/release status of (at least) this repository manually; "
          # start worker threads
          thread={}
          out={}
          for repo in repos:
            out[repo]={'success': True, 'message': ''}
            thread[repo]=threading.Thread(target=tagRefRelease, args=(repo, out[repo]))
            thread[repo].start()
          # wait for all threads
          for repo in repos:
            thread[repo].join()
          # combine output of all threads
          for repo in repos:
            if not out[repo]['success']:
              response_data['success']=False
            response_data['message']=response_data['message']+out[repo]['message']
          # set message if everything was done Successfully
          if response_data['success']==True:
            response_data['message']="Successfully released and tagged this distribution."
            # copy the distribution to the release dir
            srcFile=data['reportOutDir']+"/distribute/"+data['distArchiveName']
            dstFile="/var/www/html/mbsim/releases/"+relArchiveName
            shutil.copyfile(srcFile, dstFile)
    
    # check if the session ID provided as POST is authorizised 
    if action=="/checkmbsimenvsessionid" and method=="POST":
      with ConfigFile(False) as config: pass
      data=json.load(sys.stdin)
      if 'mbsimenvsessionid' in data:
        response_data=checkCredicals(config, data['mbsimenvsessionid'])

except:
  if __name__ == "__main__":
    # reset all output and generate a json error message
    import traceback
    defaultOutput=True
    response_data={}
    response_data['success']=False
    response_data['message']="Internal error: Please report the following error to the maintainer:\n"+traceback.format_exc()
  else:
    raise
  
if __name__ == "__main__":
  # generate response
  if defaultOutput:
    print('Content-Type: application/json')
    print('Access-Control-Allow-Credentials: true')
    print()
    print(json.dumps(response_data))

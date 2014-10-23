#! /usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import argparse
import os
from os.path import join as pj
import subprocess
import re
import glob
import sys
import datetime
import fileinput
import shutil
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
toolDependencies=dict()
toolXMLDocCopyDir=dict()
toolDoxyDocCopyDir=dict()
docDir=None
timeID=None

# command line option definition
argparser = argparse.ArgumentParser(
formatter_class=argparse.ArgumentDefaultsHelpFormatter,
description='''
Build MBSim, OpenMBV and all other tools.

After building, runexamples.py is called by this script.
All unknown options are passed to runexamples.py.
'''
)

mainOpts=argparser.add_argument_group('Main Options')
mainOpts.add_argument("--sourceDir", type=str, required=True,
  help="The base source/build directory (see --srcSuffix/--binSuffix for VPATH builds")
configOpts=mainOpts.add_mutually_exclusive_group(required=True)
configOpts.add_argument("--prefix", type=str, help="run configure using this directory as prefix option")
configOpts.add_argument("--recheck", action="store_true",
  help="run config.status --recheck instead of configure")

cfgOpts=argparser.add_argument_group('Configuration Options')
cfgOpts.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only make and runexamples.py)")
cfgOpts.add_argument("--forceBuild", default=list(), type=str, nargs="*",
  help="Force building a tool including its dependencies. Build all if no second argument is given")

cfgOpts.add_argument("--disableUpdate", action="store_true", help="Do not update using svn")
cfgOpts.add_argument("--disableConfigure", action="store_true", help="Do not manually configure. 'make' may still trigger it")
cfgOpts.add_argument("--disableMakeClean", action="store_true", help="Do not 'make clean'")
cfgOpts.add_argument("--disableMakeInstall", action="store_true", help="Do not 'make install'")
cfgOpts.add_argument("--disableMake", action="store_true", help="Do not 'make clean', 'make' and 'make install'")
cfgOpts.add_argument("--disableMakeCheck", action="store_true", help="Do not 'make check'")
cfgOpts.add_argument("--disableDoxygen", action="store_true", help="Do not build the doxygen doc")
cfgOpts.add_argument("--disableXMLDoc", action="store_true", help="Do not build the XML doc")
cfgOpts.add_argument("--disableRunExamples", action="store_true", help="Do not execute runexamples.py")
cfgOpts.add_argument("--srcSuffix", default="", help='base tool name suffix for the source dir in --sourceDir (default: "" = no VPATH build)')
cfgOpts.add_argument("--binSuffix", default="", help='base tool name suffix for the binary (build) dir in --sourceDir (default: "" = no VPATH build)')

outOpts=argparser.add_argument_group('Output Options')
outOpts.add_argument("--reportOutDir", default="build_report", type=str, help="the output directory of the report")
outOpts.add_argument("--docOutDir", type=str,
  help="Copy the documention to this directory. If not given do not copy")
outOpts.add_argument("--url", type=str, help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the RSS feed")
outOpts.add_argument("--buildType", default="", type=str, help="A description of the build type (e.g: 'Daily Build: ')")
outOpts.add_argument("--rotate", default=3, type=int, help="keep last n results and rotate them")

passOpts=argparser.add_argument_group('Options beeing passed to other commands')
passOpts.add_argument("--passToRunexamples", default=list(), nargs=argparse.REMAINDER,
  help="pass all following options, up to but not including the next --passTo* argument, to runexamples.py.")
passOpts.add_argument("--passToConfigure", default=list(), nargs=argparse.REMAINDER,
  help="pass all following options, up to but not including the next --passTo* argument, to configure.")

# parse command line options
args=argparser.parse_args() # modified by mypostargparse

# rotate
def rotateOutput():
  # create output dir
  if not os.path.isdir(args.reportOutDir): os.makedirs(args.reportOutDir)

  # get result IDs of last runs
  resultID=[]
  for curdir in glob.glob(pj(args.reportOutDir, "result_*")):
    currentID=1
    # skip all except result_[0-9]+
    try: currentID=int(curdir[len(pj(args.reportOutDir, "result_")):])
    except ValueError: continue
    # skip symbolic links
    if os.path.islink(curdir):
      os.remove(curdir)
      continue
    # add to resultID
    resultID.append(currentID);
  # sort resultID
  resultID=sorted(resultID)

  # calculate ID for this run
  if len(resultID)>0:
    currentID=resultID[-1]+1
  else:
    currentID=1

  # only keep args.rotate old results
  delFirstN=len(resultID)-args.rotate
  if delFirstN>0:
    for delID in resultID[0:delFirstN]:
      shutil.rmtree(pj(args.reportOutDir, "result_%010d"%(delID)))
    resultID=resultID[delFirstN:]

  # create link for very last result
  lastLinkID=1
  if len(resultID)>0:
    lastLinkID=resultID[0]
  try: os.remove(pj(args.reportOutDir, "result_%010d"%(lastLinkID-1)))
  except OSError: pass
  os.symlink("result_%010d"%(lastLinkID), pj(args.reportOutDir, "result_%010d"%(lastLinkID-1)))
  # create link for very first result
  try: os.remove(pj(args.reportOutDir, "result_%010d"%(currentID+1)))
  except OSError: pass
  os.symlink("result_%010d"%(currentID), pj(args.reportOutDir, "result_%010d"%(currentID+1)))
  # create link for current result
  try: os.remove(pj(args.reportOutDir, "result_current"))
  except OSError: pass
  os.symlink("result_%010d"%(currentID), pj(args.reportOutDir, "result_current"))

  # fix reportOutDir, create and clean output dir
  args.reportOutDir=pj(args.reportOutDir, "result_%010d"%(currentID))
  if os.path.isdir(args.reportOutDir): shutil.rmtree(args.reportOutDir)
  os.makedirs(args.reportOutDir)

# the main routine being called ones
def main():
  mypostargparse(args)
  args.sourceDir=os.path.abspath(args.sourceDir)
  args.reportOutDir=os.path.abspath(args.reportOutDir)

  # all tools to be build including the tool dependencies
  global toolDependencies
  global toolXMLDocCopyDir
  global toolDoxyDocCopyDir

  toolDependencies={
    #   |ToolName   |WillFail (if WillFail is true no RSS Feed error is reported if this Tool fails somehow)
    pj('fmatvec'): [False, set([ # depends on
      ])],
    pj('hdf5serie', 'h5plotserie'): [False, set([ # depends on
        pj('hdf5serie', 'hdf5serie')
      ])],
    pj('hdf5serie', 'hdf5serie'): [False, set([ # depends on
        pj('fmatvec')
      ])],
    pj('openmbv', 'mbxmlutils'): [False, set([ # depends on
      ])],
    pj('openmbv', 'openmbv'): [False, set([ # depends on
        pj('openmbv', 'openmbvcppinterface'),
        pj('hdf5serie', 'hdf5serie')
      ])],
    pj('openmbv', 'openmbvcppinterface'): [False, set([ # depends on
        pj('hdf5serie', 'hdf5serie'),
        pj('openmbv', 'mbxmlutils')
      ])],
    pj('mbsim', 'kernel'): [False, set([ # depends on
        pj('fmatvec'),
        pj('openmbv', 'openmbvcppinterface')
      ])],
    pj('mbsim', 'modules', 'mbsimHydraulics'): [False, set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'modules', 'mbsimControl')
      ])],
    pj('mbsim', 'modules', 'mbsimFlexibleBody'): [False, set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'thirdparty', 'nurbs++')
      ])],
    pj('mbsim', 'thirdparty', 'nurbs++'): [False, set([ # depends on
      ])],
    pj('mbsim', 'modules', 'mbsimPowertrain'): [False, set([ # depends on
        pj('mbsim', 'kernel')
      ])],
    pj('mbsim', 'modules', 'mbsimElectronics'): [False, set([ # depends on
        pj('mbsim', 'kernel')
      ])],
    pj('mbsim', 'modules', 'mbsimControl'): [False, set([ # depends on
        pj('mbsim', 'kernel')
      ])],
    pj('mbsim', 'modules', 'mbsimInterface'): [False, set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'modules', 'mbsimControl')
      ])],
    pj('mbsim', 'mbsimxml'): [False, set([ # depends on
        pj('mbsim', 'kernel'),
        pj('openmbv', 'openmbvcppinterface'),
        pj('openmbv', 'mbxmlutils'),
        # dependencies to mbsim modules (plugins) are only required for correct xmldoc generation 
        pj('mbsim', 'modules', 'mbsimHydraulics'),
        pj('mbsim', 'modules', 'mbsimFlexibleBody'),
        pj('mbsim', 'modules', 'mbsimPowertrain'),
        pj('mbsim', 'modules', 'mbsimElectronics'),
        pj('mbsim', 'modules', 'mbsimControl'),
        pj('mbsim', 'modules', 'mbsimInterface')
      ])],
    pj('mbsim', 'mbsimgui'): [False, set([ # depends on
        pj('openmbv', 'openmbv'),
        pj('openmbv', 'mbxmlutils'),
        pj('mbsim', 'mbsimxml')
      ])],
    pj('mbsim', 'examples'): [False, set([ # depends on
        pj('mbsim', 'mbsimxml'),
        pj('mbsim', 'kernel'),
        pj('mbsim', 'modules', 'mbsimHydraulics'),
        pj('mbsim', 'modules', 'mbsimFlexibleBody'),
        pj('mbsim', 'modules', 'mbsimPowertrain'),
        pj('mbsim', 'modules', 'mbsimElectronics'),
        pj('mbsim', 'modules', 'mbsimControl'),
        pj('mbsim', 'modules', 'mbsimInterface')
      ])]
  }
  toolXMLDocCopyDir={
    pj("mbsim", "kernel"):                       set(["http___mbsim_berlios_de_MBSim", "http___mbsim_berlios_de_MBSimIntegrator"]),
    pj("mbsim", "modules", "mbsimFlexibleBody"): set(["http___mbsim_berlios_de_MBSimFlexibleBody"]),
    pj("mbsim", "modules", "mbsimControl"):      set(["http___mbsim_berlios_de_MBSimControl"]),
    pj("mbsim", "modules", "mbsimInterface"):    set(["http___mbsim_berlios_de_MBSimInterface"]),
    pj("mbsim", "modules", "mbsimHydraulics"):   set(["http___mbsim_berlios_de_MBSimHydraulics"]),
    pj("mbsim", "modules", "mbsimPowertrain"):   set(["http___mbsim_berlios_de_MBSimPowertrain"]),
    pj("mbsim", "mbsimxml"):                     set(["http___mbsim_berlios_de_MBSimXML"]),
    pj("openmbv", "mbxmlutils"):                 set(["http___openmbv_berlios_de_MBXMLUtils_physicalvariable"]),
    pj("openmbv", "openmbv"):                    set(["http___openmbv_berlios_de_OpenMBV"])
  }
  toolDoxyDocCopyDir={
    pj("fmatvec"):                               set(["fmatvec"]),
    pj("hdf5serie", "hdf5serie"):                set([pj("hdf5serie", "html")]),
    pj("openmbv", "openmbvcppinterface"):        set([pj("openmbvcppinterface", "html")]),
    pj("openmbv", "mbxmlutils"):                 set([pj("mbxmlutils", "html")]),
    pj("mbsim", "kernel"):                       set(["mbsim"]),
    pj("mbsim", "modules", "mbsimFlexibleBody"): set(["mbsimflexiblebody"]),
    pj("mbsim", "modules", "mbsimControl"):      set(["mbsimcontrol"]),
    pj("mbsim", "modules", "mbsimInterface"):    set(["mbsiminterface"]),
    pj("mbsim", "modules", "mbsimElectronics"):  set(["mbsimelectronics"]),
    pj("mbsim", "modules", "mbsimHydraulics"):   set(["mbsimhydraulics"]),
    pj("mbsim", "modules", "mbsimPowertrain"):   set(["mbsimpowertrain"])
  }

  # extend the dependencies recursively
  addAllDepencencies()

  # set docDir
  global docDir
  if args.prefix==None:
    output=subprocess.check_output([pj(args.sourceDir, "openmbv"+args.binSuffix, "mbxmlutils", "config.status"), "--config"]).decode("utf-8")
    for opt in output.split():
      match=re.search("'?--prefix[= ]([^']*)'?", opt)
      if match!=None:
        docDir=pj(match.expand("\\1"), "share", "mbxmlutils", "doc")
        break
  else:
    docDir=pj(args.prefix, "share", "mbxmlutils", "doc")
  # append path to PKG_CONFIG_PATH to find mbxmlutils and co. by runexmaples.py
  pkgConfigDir=os.path.normpath(pj(docDir, os.pardir, os.pardir, os.pardir, "lib", "pkgconfig"))
  if "PKG_CONFIG_PATH" in os.environ:
    os.environ["PKG_CONFIG_PATH"]=pkgConfigDir+os.pathsep+os.environ["PKG_CONFIG_PATH"]
  else:
    os.environ["PKG_CONFIG_PATH"]=pkgConfigDir

  global timeID
  timeID=datetime.datetime.now()
  timeID=datetime.datetime(timeID.year, timeID.month, timeID.day, timeID.hour, timeID.minute, timeID.second)
  # write main doc file
  if args.docOutDir!=None:
    args.docOutDir=os.path.abspath(args.docOutDir)
    if not os.path.isdir(pj(args.docOutDir, "xmldoc")): os.makedirs(pj(args.docOutDir, "xmldoc"))
    if not os.path.isdir(pj(args.docOutDir, "doc")): os.makedirs(pj(args.docOutDir, "doc"))
    # create doc entry html
    docFD=open(pj(args.docOutDir, "index.html"), "w")
    print('<!DOCTYPE html>', file=docFD)
    print('<html lang="en">', file=docFD)
    print('<head>', file=docFD)
    print('  <title>MBSim, OpenMBV, ... Documentation</title>', file=docFD)
    print('  <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>', file=docFD)
    print('</head>', file=docFD)
    print('<body style="margin:1em">', file=docFD)
    print('<h1>MBSim, OpenMBV, ... Documentation</h1>', file=docFD)
    print('<p>Generated at: %s</p>'%(str(timeID)), file=docFD)
    print('<div class="panel panel-success">', file=docFD)
    print('  <div class="panel-heading">XML Documentation</div>', file=docFD)
    print('  <ul class="list-group">', file=docFD)
    print('    <li class="list-group-item"><a href="'+myurllib.pathname2url(pj("xmldoc", "http___mbsim_berlios_de_MBSimXML", "mbsimxml.html"))+'">MBSimXML</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</div>', file=docFD)
    print('<div class="panel panel-info">', file=docFD)
    print('  <div class="panel-heading">Doxygen Documentation</div>', file=docFD)
    print('  <ul class="list-group">', file=docFD)
    for d in sorted(list(toolDoxyDocCopyDir)):
      print('    <li class="list-group-item"><a href="'+myurllib.pathname2url(pj("doc", d, "index.html"))+'">'+d+'</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</div>', file=docFD)
    print('</body>', file=docFD)
    print('</html>', file=docFD)
    docFD.close()

  # start messsage
  print("Started build process.")
  print("See the log file "+pj(args.reportOutDir, "result_current", "index.html")+" for detailed results.")
  if args.docOutDir!=None:
    print("See also the generated documentation "+pj(args.docOutDir, "index.html")+".\n")

  # rotate (modifies args.reportOutDir)
  rotateOutput()

  # svn update all tools
  buildTools=set()
  updateFailed=set()
  nr=1
  for tool in toolDependencies:
    if update(nr, tool, buildTools)!=0:
      updateFailed.add(tool)
    nr+=1
  updatedTools=buildTools.copy()

  # force build
  for i, value in enumerate(args.forceBuild): # normalize all given path
    args.forceBuild[i]=os.path.normpath(value)
  if len(args.forceBuild)==0:
    args.forceBuild.extend(list(toolDependencies))
  buildTools.update(args.forceBuild)

  # a list of all tools to be build
  allBuildTools(buildTools)

  # a sorted list of all tools te be build (in the correct order according the dependencies)
  orderedBuildTools=list()
  sortBuildTools(buildTools, orderedBuildTools)

  # write empty RSS feed
  writeRSSFeed(0) # nrFailed == 0 => write empty RSS feed

  # create index.html
  mainFD=open(pj(args.reportOutDir, "index.html"), "w")
  print('<!DOCTYPE html>', file=mainFD)
  print('<html lang="en">', file=mainFD)
  print('<head>', file=mainFD)
  print('  <title>MBSim, OpenMBV, ... Build Results</title>', file=mainFD)
  print('  <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>', file=mainFD)
  print('  <link rel="stylesheet" href="http://cdn.datatables.net/1.10.2/css/jquery.dataTables.css"/>', file=mainFD)
  print('  <link rel="alternate" type="application/rss+xml" title="MBSim, OpenMBV, ... Build Results" href="../result.rss.xml"/>', file=mainFD)
  print('</head>', file=mainFD)
  print('<body style="margin:1em">', file=mainFD)
  print('<script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>', file=mainFD)
  print('<script type="text/javascript" src="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/js/bootstrap.min.js"> </script>', file=mainFD)
  print('<script type="text/javascript" src="http://cdn.datatables.net/1.10.2/js/jquery.dataTables.min.js"> </script>', file=mainFD)
  print('<script type="text/javascript">', file=mainFD)
  print('  $(document).ready(function() {', file=mainFD)
  print("    $.fn.dataTableExt.sErrMode = 'throw';", file=mainFD)
  print("    $('#SortThisTable').dataTable({'lengthMenu': [ [1, 5, 10, 25, -1], [1, 5, 10, 25, 'All'] ], 'pageLength': -1, 'aaSorting': [], stateSave: true});", file=mainFD)
  print('  } );', file=mainFD)
  print('</script>', file=mainFD)

  print('<h1>MBSim, OpenMBV, ... Build Results</h1>', file=mainFD)

  print('<dl class="dl-horizontal">', file=mainFD)
  print('  <dt>Called command</dt><dd><code>', file=mainFD)
  for argv in sys.argv: print(argv+' ', file=mainFD)
  print('  </code></dd>', file=mainFD)
  print('  <dt>RSS Feed</dt><dd>Use the feed "auto-discovery" of this page or click <a href="../result.rss.xml">here</a></dd>', file=mainFD)
  print('  <dt>Time ID</dt><dd>'+str(timeID)+'</dd>', file=mainFD)
  print('  <dt>End time</dt><dd><span id="STILLRUNNINGORABORTED" class="text-danger"><b>still running or aborted</b></span><!--E_ENDTIME--></dd>', file=mainFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  print('  <dt>Navigate</dt><dd><a class="btn btn-info btn-xs" href="../result_%010d/index.html"><span class="glyphicon glyphicon-step-backward"> </span> previous</a>'%(currentID-1), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="../result_%010d/index.html"><span class="glyphicon glyphicon-step-forward"> </span> next</a>'%(currentID+1), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="../result_current/index.html"><span class="glyphicon glyphicon-fast-forward"> </span> newest</a>', file=mainFD)
  print('                    </dd>', file=mainFD)
  print('</dl>', file=mainFD)

  print('<hr/><p><span class="glyphicon glyphicon-info-sign"> </span> Failures in the following table should be fixed from top to bottom since a error in one tool may cause errors on dependent tools.<br/>', file=mainFD)
  print('<span class="glyphicon glyphicon-info-sign"> </span> A tool name in gray color is a tool which may fail and is therefore not reported as an error in the RSS feed.</p>', file=mainFD)

  print('<table id="SortThisTable" class="table table-striped table-hover table-bordered compact">', file=mainFD)
  print('<thead><tr>', file=mainFD)
  print('<th>Tool</th>', file=mainFD)
  print('<th>SVN Update</th>', file=mainFD)
  print('<th>Configure</th>', file=mainFD)
  print('<th>Make</th>', file=mainFD)
  print('<th>Check</th>', file=mainFD)
  print('<th>Doxygen Doc.</th>', file=mainFD)
  print('<th>XML Doc.</th>', file=mainFD)
  print('</tr></thead></tbody>', file=mainFD)

  # list tools which are not updated and must not be rebuild according dependencies
  for tool in set(toolDependencies)-set(orderedBuildTools):
    print('<tr>', file=mainFD)
    print('<td>'+tool+'</td>', file=mainFD)
    print('<td class="success"><a href="'+myurllib.pathname2url(pj(tool, "svn.txt"))+'">up to date, no rebuild required</a></td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('</tr>', file=mainFD)
  mainFD.flush()

  # build the other tools in order
  ret=0
  retRunExamples=0
  nr=1
  for tool in orderedBuildTools:
    r1, r2=build(nr, len(orderedBuildTools), tool, mainFD, updatedTools, updateFailed)
    if toolDependencies[tool][0]==False:
      ret+=r1
      retRunExamples+=r2
    nr+=1

  print('</tbody></table>', file=mainFD)
  print('</body>', file=mainFD)
  print('</html>', file=mainFD)

  mainFD.close()
  # replace <span id="STILLRUNNINGORABORTED"...</span> in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    endTime=datetime.datetime.now()
    endTime=datetime.datetime(endTime.year, endTime.month, endTime.day, endTime.hour, endTime.minute, endTime.second)
    line=re.sub('<span id="STILLRUNNINGORABORTED".*?</span>', str(endTime), line)
    print(line)

  # write RSS feed
  writeRSSFeed(ret)

  return ret+retRunExamples



#####################################################################################
# from now on only functions follow and at the end main is called
#####################################################################################



def addAllDepencencies():
  rec=False
  for t in toolDependencies:
    add=set()
    oldLength=len(toolDependencies[t][1])
    for d in toolDependencies[t][1]:
      for a in toolDependencies[d][1]:
        add.add(a)
    for a in add:
      toolDependencies[t][1].add(a)
    newLength=len(toolDependencies[t][1])
    if newLength>oldLength:
      rec=True
  if rec:
    addAllDepencencies()


 
def allBuildTools(buildTools):
  add=set()
  for bt in buildTools:
    for t in toolDependencies:
      if bt in toolDependencies[t][1]:
        add.add(t)
  buildTools.update(add)



def sortBuildTools(buildTools, orderedBuildTools):
  upToDate=set(toolDependencies)-buildTools
  for bt in buildTools:
    if len(toolDependencies[bt][1]-upToDate)==0:
      orderedBuildTools.append(bt)
      upToDate.add(bt)
  buildTools-=set(orderedBuildTools)
  if len(buildTools)>0:
    sortBuildTools(buildTools, orderedBuildTools)



def srcTool(tool):
  t=tool.split(os.path.sep)
  t[0]=t[0]+args.srcSuffix
  return os.path.sep.join(t)
def buildTool(tool):
  t=tool.split(os.path.sep)
  t[0]=t[0]+args.binSuffix
  return os.path.sep.join(t)

def update(nr, tool, buildTools):
  savedDir=os.getcwd()
  os.chdir(pj(args.sourceDir, srcTool(tool)))

  # write svn output to report dir
  if not os.path.isdir(pj(args.reportOutDir, tool)): os.makedirs(pj(args.reportOutDir, tool))
  svnFD=open(pj(args.reportOutDir, tool, "svn.txt"), "w")
  print("stderr output:", file=svnFD)
  print("", file=svnFD)

  ret=0
  if not args.disableUpdate:
    print("Updating "+str(nr)+"/"+str(len(toolDependencies))+": "+tool)

    try:
      output=subprocess.check_output(["svn", "update", "--non-interactive"], stderr=svnFD)
    except subprocess.CalledProcessError as ex:
      output=b""
      ret=1
    if re.search("\nUpdated to revision [0-9]+.\n", output.decode("utf-8"))!=None:
      buildTools.add(tool)
    if re.search("\nSummary of conflicts:\n", output.decode("utf-8"))!=None:
      ret=1
  else:
    output=b"Update disabled"
    
  print("", file=svnFD)
  print("", file=svnFD)
  print("stdout output:", file=svnFD)
  print("", file=svnFD)
  print(output.decode("utf-8"), file=svnFD)
  svnFD.close()

  os.chdir(savedDir)
  return ret



def build(nr, nrAll, tool, mainFD, updatedTools, updateFailed):
  print("Building "+str(nr)+"/"+str(nrAll)+": "+tool+": ", end=""); sys.stdout.flush()

  ret=0
  retRunExamples=0

  # print svn update
  if toolDependencies[tool][0]==False:
    print('<tr>', file=mainFD)
  else:
    print('<tr class="text-muted">', file=mainFD)
  print('<td>'+tool+'</td>', file=mainFD)
  if tool in updateFailed:
    print('<td class="danger"><a href="'+myurllib.pathname2url(pj(tool, "svn.txt"))+'">failed</a></td>', file=mainFD)
    ret+=1
  else:
    if tool in updatedTools:
      print('<td class="success"><a href="'+myurllib.pathname2url(pj(tool, "svn.txt"))+'">updated, rebuild required</a></td>', file=mainFD)
    else:
      print('<td class="success"><a href="'+myurllib.pathname2url(pj(tool, "svn.txt"))+'">up to date, rebuild required</a></td>', file=mainFD)
  mainFD.flush()

  savedDir=os.getcwd()
  if tool==pj("mbsim", "examples"):
    os.chdir(pj(args.sourceDir, srcTool(tool)))
    print("runexamples.py", end=""); sys.stdout.flush()
    retRunExamples+=runexamples(mainFD)
  else:
    # configure
    print("configure", end=""); sys.stdout.flush()
    ret+=configure(tool, mainFD)

    # cd to build dir
    os.chdir(savedDir)
    os.chdir(pj(args.sourceDir, buildTool(tool)))

    # make
    print(", make", end=""); sys.stdout.flush()
    ret+=make(tool, mainFD)

    # make check
    print(", check", end=""); sys.stdout.flush()
    ret+=check(tool, mainFD)

    # doxygen
    print(", doxygen-doc", end=""); sys.stdout.flush()
    ret+=doc(tool, mainFD, args.disableDoxygen, "doc", toolDoxyDocCopyDir)

    # xmldoc
    print(", xml-doc", end=""); sys.stdout.flush()
    ret+=doc(tool, mainFD, args.disableXMLDoc, "xmldoc", toolXMLDocCopyDir)
  os.chdir(savedDir)

  print("")
  print('</tr>', file=mainFD)
  mainFD.flush()

  return ret, retRunExamples



def configure(tool, mainFD):
  configureFD=open(pj(args.reportOutDir, tool, "configure.txt"), "w")
  copyConfigLog=False
  savedDir=os.getcwd()
  try:
    if not args.disableConfigure:
      # pre configure
      os.chdir(pj(args.sourceDir, srcTool(tool)))
      print("\n\nRUNNING aclocal\n", file=configureFD); configureFD.flush()
      if subprocess.call(["aclocal"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("aclocal failed")
      print("\n\nRUNNING autoheader\n", file=configureFD); configureFD.flush()
      if subprocess.call(["autoheader"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("autoheader failed")
      print("\n\nRUNNING libtoolize\n", file=configureFD); configureFD.flush()
      if subprocess.call(["libtoolize", "-c"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("libtoolize failed")
      print("\n\nRUNNING automake\n", file=configureFD); configureFD.flush()
      if subprocess.call(["automake", "-a", "-c"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("automake failed")
      print("\n\nRUNNING autoconf\n", file=configureFD); configureFD.flush()
      if subprocess.call(["autoconf"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("autoconf failed")
      print("\n\nRUNNING autoreconf\n", file=configureFD); configureFD.flush()
      if subprocess.call(["autoreconf"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("autoreconf failed")
      # configure
      os.chdir(savedDir)
      os.chdir(pj(args.sourceDir, buildTool(tool)))
      copyConfigLog=True
      print("\n\nRUNNING configure\n", file=configureFD); configureFD.flush()
      if args.prefix==None:
        if subprocess.call(["./config.status", "--recheck"], stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("configure failed")
      else:
        command=[pj(args.sourceDir, srcTool(tool), "configure"), "--prefix", args.prefix]
        command.extend(args.passToConfigure)
        if subprocess.call(command, stderr=subprocess.STDOUT, stdout=configureFD)!=0: raise RuntimeError("configure failed")
    else:
      print("configure disabled", file=configureFD); configureFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  print('<td class="%s">'%("success" if result=="done" else "danger"), file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, "configure.txt"))+'">'+result+'</a>', file=mainFD)
  if copyConfigLog:
    shutil.copyfile("config.log", pj(args.reportOutDir, tool, "config.log.txt"))
    print('  <a href="'+myurllib.pathname2url(pj(tool, "config.log.txt"))+'">config.log</a>', file=mainFD)
  print('</td>', file=mainFD)
  configureFD.close()
  mainFD.flush()
  os.chdir(savedDir)

  if result!="done":
    return 1
  return 0



def make(tool, mainFD):
  makeFD=open(pj(args.reportOutDir, tool, "make.txt"), "w")
  try:
    if not args.disableMake:
      # make
      errStr=""
      if not args.disableMakeClean:
        print("\n\nRUNNING make clean\n", file=makeFD); makeFD.flush()
        if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=makeFD)!=0: errStr=errStr+"make clean failed; "
      print("\n\nRUNNING make -k\n", file=makeFD); makeFD.flush()
      if subprocess.call(["make", "-k", "-j", str(args.j)], stderr=subprocess.STDOUT, stdout=makeFD)!=0: errStr=errStr+"make failed; "
      if not args.disableMakeInstall:
        print("\n\nRUNNING make install\n", file=makeFD); makeFD.flush()
        if subprocess.call(["make", "-k", "install"], stderr=subprocess.STDOUT, stdout=makeFD)!=0: errStr=errStr+"make install failed; "
      if errStr!="": raise RuntimeError(errStr)
    else:
      print("make disabled", file=makeFD); makeFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  print('<td class="%s">'%("success" if result=="done" else "danger"), file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, "make.txt"))+'">'+result+'</a>', file=mainFD)
  print('</td>', file=mainFD)
  makeFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def check(tool, mainFD):
  checkFD=open(pj(args.reportOutDir, tool, "check.txt"), "w")
  if not args.disableMakeCheck:
    # make check
    print("RUNNING make check\n", file=checkFD); checkFD.flush()
    if subprocess.call(["make", "-j", str(args.j), "check"], stderr=subprocess.STDOUT, stdout=checkFD)==0:
      result="done"
    else:
      result="failed"
  else:
    print("make check disabled", file=checkFD); checkFD.flush()
    result="done"

  foundTestSuiteLog=False
  testSuiteLogFD=open(pj(args.reportOutDir, tool, "test-suite.log.txt"), "w")
  for rootDir,_,files in os.walk('.'): # append all test-suite.log files
    if "test-suite.log" in files:
      testSuiteLogFD.write('\n\n')
      testSuiteLogFD.write(open(pj(rootDir, "test-suite.log")).read())
      foundTestSuiteLog=True
  testSuiteLogFD.close()
  if not args.disableMakeCheck:
    print('<td class="%s">'%("success" if result=="done" else "danger"), file=mainFD)
    print('  <a href="'+myurllib.pathname2url(pj(tool, "check.txt"))+'">'+result+'</a>', file=mainFD)
    if foundTestSuiteLog:
      print('  <a href="'+myurllib.pathname2url(pj(tool, "test-suite.log.txt"))+'">test-suite.log</a>', file=mainFD)
    print('</td>', file=mainFD)
  else:
    print('<td>not run</td>', file=mainFD)
  checkFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def doc(tool, mainFD, disabled, docDirName, toolDocCopyDir):
  if not os.path.isdir(docDirName):
    print('<td>not available</td>', file=mainFD)
    mainFD.flush()
    return 0

  docFD=open(pj(args.reportOutDir, tool, docDirName+".txt"), "w")
  savedDir=os.getcwd()
  os.chdir(docDirName)
  try:
    if not disabled:
      # make doc
      errStr=""
      print("\n\nRUNNING make clean\n", file=docFD); docFD.flush()
      if subprocess.call(["make", "clean"], stderr=subprocess.STDOUT, stdout=docFD)!=0: errStr=errStr+"make clean failed; "
      print("\n\nRUNNING make\n", file=docFD); docFD.flush()
      if subprocess.call(["make", "-k"], stderr=subprocess.STDOUT, stdout=docFD)!=0: errStr=errStr+"make failed; "
      print("\n\nRUNNING make install\n", file=docFD); docFD.flush()
      if subprocess.call(["make", "-k", "install"], stderr=subprocess.STDOUT, stdout=docFD)!=0: errStr=errStr+"make install failed; "
      if errStr!="": raise RuntimeError(errStr)

      # copy doc
      if args.docOutDir!=None:
        if tool in toolDocCopyDir:
          for d in toolDocCopyDir[tool]:
            if docDirName=="xmldoc":
              srcInfix=os.curdir
              dstDir=d
            else:
              srcInfix=pj(os.pardir, os.pardir, "doc")
              dstDir=tool
            if os.path.isdir(pj(args.docOutDir, docDirName, dstDir)): shutil.rmtree(pj(args.docOutDir, docDirName, dstDir))
            shutil.copytree(os.path.normpath(pj(docDir, srcInfix, d)), pj(args.docOutDir, docDirName, dstDir), symlinks=True)
    else:
      print(docDirName+" disabled", file=docFD); docFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  finally:
    os.chdir(savedDir)
  print('<td class="%s">'%("success" if result=="done" else "danger"), file=mainFD)
  print('  <a href="'+myurllib.pathname2url(pj(tool, docDirName+".txt"))+'">'+result+'</a>', file=mainFD)
  print('</td>', file=mainFD)
  docFD.close()
  mainFD.flush()

  if result!="done":
    return 1
  return 0



def runexamples(mainFD):
  if args.disableRunExamples:
    print('<td>runexamples disabled</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    print('<td>-</td>', file=mainFD)
    mainFD.flush()
    return 0

  # runexamples.py command
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  command=["./runexamples.py", "-j", str(args.j)]
  if args.url!=None:
    command.extend(["--url", args.url+"/result_%010d/runexamples_report"%(currentID)])
  if args.buildType!="":
    command.extend(["--buildType", args.buildType])
  command.extend(["--reportOutDir", pj(args.reportOutDir, "runexamples_report")])
  command.extend(["--currentID", str(currentID)])
  command.extend(["--timeID", timeID.strftime("%Y-%m-%dT%H:%M:%S")])
  command.extend(args.passToRunexamples)

  print("")
  print("")
  print("Output of runexamples.py")
  print("")
  ret=subprocess.call(command, stderr=subprocess.STDOUT)

  if ret==0:
    print('<td class="success"><a href="'+myurllib.pathname2url(pj("runexamples_report", "result_current", "index.html"))+
      '">all examples passed</a></td>', file=mainFD)
  else:
    print('<td class="danger"><a href="'+myurllib.pathname2url(pj("runexamples_report", "result_current", "index.html"))+
      '">examples failed</a></td>', file=mainFD)
  print('<td>-</td>', file=mainFD)
  print('<td>-</td>', file=mainFD)
  print('<td>-</td>', file=mainFD)
  print('<td>-</td>', file=mainFD)

  mainFD.flush()

  return ret



def writeRSSFeed(nrFailed):
  rssFN="result.rss.xml"
  rssFD=open(pj(args.reportOutDir, os.pardir, rssFN), "w")
  print('''\
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>%sMBSim, OpenMBV, ... Build Results</title>
    <link>%s/result_current/index.html</link>
    <description>%sResult RSS feed of the last build of MBSim and Co.</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="%s/%s" rel="self" type="application/rss+xml"/>'''%(args.buildType, args.url, args.buildType, args.url, rssFN), file=rssFD)
  if nrFailed>0:
    currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
    print('''\
    <item>
      <title>%sBuild failed</title>
      <link>%s/result_%010d/index.html</link>
      <guid isPermaLink="false">%s/result_%010d/rss_id_%s</guid>
      <pubDate>%s</pubDate>
    </item>'''%(args.buildType,
           args.url, currentID,
           args.url, currentID, datetime.datetime.utcnow().strftime("%s"),
           datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S +0000")), file=rssFD)
  print('''\
    <item>
      <title>%sDummy feed item. Just ignore it.</title>
      <link>%s/result_current/index.html</link>
      <guid isPermaLink="false">%s/result_current/rss_id_1359206848</guid>
      <pubDate>Sat, 26 Jan 2013 14:27:28 +0000</pubDate>
    </item>
  </channel>
</rss>'''%(args.buildType, args.url, args.url), file=rssFD)
  rssFD.close()



# split mulitple "nargs=argparse.REMAINDER" arguments to the corresponding ones
def mypostargparse(args):
  # get all passTo* args
  passArgNames=list()
  for argname in args.__dict__:
    if argname.find("passTo")==0:
      passArgNames.append(argname)

  runAgain=False
  for argname in passArgNames:
    value=eval("args."+argname)
    if value!=None:
      for argname2 in passArgNames:
        if "--"+argname2 in value:
          runAgain=True
          exec('args.'+argname+'=value[0:value.index("--"+argname2)]')
          exec('args.'+argname2+'=value[value.index("--"+argname2)+1:]')

  if runAgain:
    mypostargparse(args)



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)

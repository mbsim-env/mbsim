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
import codecs
import simplesandbox
import buildSystemState
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
scriptdir=os.path.dirname(os.path.realpath(__file__))
toolDependencies=dict()
toolXMLDocCopyDir=dict()
toolDoxyDocCopyDir=dict()
docDir=None
timeID=None
args=None

# pass these envvar to simplesandbox.call
simplesandboxEnvvars=["PKG_CONFIG_PATH", "CXXFLAGS", "CFLAGS", "FFLAGS", # general required envvars
                      "LD_LIBRARY_PATH", # Linux specific required envvars
                      "WINEPATH", "PLATFORM", "CXX", "MOC", "UIC", "RCC"] # Windows specific required envvars

def parseArguments():
  # command line option definition
  argparser=argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    description='''Building the MBSim-Environment.
  
  After building, runexamples.py is called by this script.
  All unknown options are passed to runexamples.py.'''
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
    help="Force building a tool including its dependencies. Build all, the default, if no second argument is given")
  
  cfgOpts.add_argument("--enableCleanPrefix", action="store_true", help="Remove the prefix dir completely before starting")
  cfgOpts.add_argument("--disableUpdate", action="store_true", help="Do not update repositories")
  cfgOpts.add_argument("--disableConfigure", action="store_true", help="Do not manually configure. 'make' may still trigger it")
  cfgOpts.add_argument("--disableMakeClean", action="store_true", help="Do not 'make clean'")
  cfgOpts.add_argument("--disableMakeInstall", action="store_true", help="Do not 'make install'")
  cfgOpts.add_argument("--disableMake", action="store_true", help="Do not 'make clean', 'make' and 'make install'")
  cfgOpts.add_argument("--disableMakeCheck", action="store_true", help="Do not 'make check'")
  cfgOpts.add_argument("--disableDoxygen", action="store_true", help="Do not build the doxygen doc")
  cfgOpts.add_argument("--disableXMLDoc", action="store_true", help="Do not build the XML doc")
  cfgOpts.add_argument("--disableRunExamples", action="store_true", help="Do not execute runexamples.py")
  cfgOpts.add_argument("--enableDistribution", action="store_true", help="Create a release distribution archive (only usefull on the buildsystem)")
  cfgOpts.add_argument("--srcSuffix", default="", help='base tool name suffix for the source dir in --sourceDir (default: "" = no VPATH build)')
  cfgOpts.add_argument("--binSuffix", default="", help='base tool name suffix for the binary (build) dir in --sourceDir (default: "" = no VPATH build)')
  cfgOpts.add_argument("--fmatvecBranch", default="", help='In the fmatvec repo checkout the branch FMATVECBRANCH')
  cfgOpts.add_argument("--hdf5serieBranch", default="", help='In the hdf5serierepo checkout the branch HDF5SERIEBRANCH')
  cfgOpts.add_argument("--openmbvBranch", default="", help='In the openmbv repo checkout the branch OPENMBVBRANCH')
  cfgOpts.add_argument("--mbsimBranch", default="", help='In the mbsim repo checkout the branch MBSIMBRANCH')
  cfgOpts.add_argument("--buildSystemRun", action="store_true", help='Run in build system mode: generate build system state files and run with simplesandbox.')
  
  outOpts=argparser.add_argument_group('Output Options')
  outOpts.add_argument("--reportOutDir", default="build_report", type=str, help="the output directory of the report")
  outOpts.add_argument("--docOutDir", type=str,
    help="Copy the documention to this directory. If not given do not copy")
  outOpts.add_argument("--url", type=str, help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the Atom feed")
  outOpts.add_argument("--buildType", default="local", type=str, help="A description of the build type (e.g: linux64-dailydebug)")
  outOpts.add_argument("--rotate", default=3, type=int, help="keep last n results and rotate them")
  
  passOpts=argparser.add_argument_group('Options beeing passed to other commands')
  passOpts.add_argument("--passToRunexamples", default=list(), nargs=argparse.REMAINDER,
    help="pass all following options, up to but not including the next --passToConfigure argument, to runexamples.py.")
  passOpts.add_argument("--passToConfigure", default=list(), nargs=argparse.REMAINDER,
    help="pass all following options, up to but not including the next --passToRunexamples argument, to configure.")
  
  # parse command line options:
   
  # parse all options before --passToConfigure and/or --passToRunexamples by argparser. 
  passTo1=min(sys.argv.index("--passToConfigure"  ) if "--passToConfigure"   in sys.argv else len(sys.argv),
              sys.argv.index("--passToRunexamples") if "--passToRunexamples" in sys.argv else len(sys.argv))
  passTo2=max(sys.argv.index("--passToConfigure"  ) if "--passToConfigure"   in sys.argv else len(sys.argv),
              sys.argv.index("--passToRunexamples") if "--passToRunexamples" in sys.argv else len(sys.argv))
  global args
  args=argparser.parse_args(args=sys.argv[1:passTo1]) # do not pass REMAINDER arguments
  # assign the options after --passToConfigure and/or --passToRunexamples to args.passTo...
  if "--passToConfigure" in sys.argv[passTo1:passTo2]:
    args.passToConfigure=sys.argv[passTo1+1:passTo2]
  if "--passToRunexamples" in sys.argv[passTo1:passTo2]:
    args.passToRunexamples=sys.argv[passTo1+1:passTo2]
  if "--passToConfigure" in sys.argv[passTo2:]:
    args.passToConfigure=sys.argv[passTo2+1:]
  if "--passToRunexamples" in sys.argv[passTo2:]:
    args.passToRunexamples=sys.argv[passTo2+1:]

def htmlEscape(text):
  htmlEscapeTable={
    "&": "&amp;",
    '"': "&quot;",
    "'": "&apos;",
    ">": "&gt;",
    "<": "&lt;",
  }
  return "".join(htmlEscapeTable.get(c,c) for c in text)

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
  parseArguments()
  args.sourceDir=os.path.abspath(args.sourceDir)
  args.reportOutDir=os.path.abspath(args.reportOutDir)

  # all tools to be build including the tool dependencies
  global toolDependencies
  global toolXMLDocCopyDir
  global toolDoxyDocCopyDir

  toolDependencies={
    #   |ToolName   |WillFail (if WillFail is true no Atom Feed error is reported if this Tool fails somehow)
    pj('fmatvec'): [False, set([ # depends on
      ])],
    pj('hdf5serie', 'h5plotserie'): [False, set([ # depends on
        pj('hdf5serie', 'hdf5serie')
      ])],
    pj('hdf5serie', 'hdf5serie'): [False, set([ # depends on
        pj('fmatvec')
      ])],
    pj('openmbv', 'mbxmlutils'): [False, set([ # depends on
        pj('fmatvec')
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
        pj('mbsim', 'kernel'),
        pj('mbsim', 'modules', 'mbsimControl')
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
    pj('mbsim', 'mbsimfmi'): [False, set([ # depends on
        pj('mbsim', 'kernel'),
        pj('mbsim', 'mbsimxml'),
        pj('mbsim', 'modules', 'mbsimControl')
      ])],
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
    raise RuntimeError("MISSING: calling without --prefix is currently not supported. sandboxing is missing")
    output=subprocess.check_output([pj(args.sourceDir, "openmbv"+args.binSuffix, "mbxmlutils", "config.status"), "--config"]).decode("utf-8")
    for opt in output.split():
      match=re.search("'?--prefix[= ]([^']*)'?", opt)
      if match!=None:
        docDir=pj(match.expand("\\1"), "share", "mbxmlutils", "doc")
        args.prefixAuto=match.expand("\\1")
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
    docFD=codecs.open(pj(args.docOutDir, "index.html"), "w", encoding="utf-8")
    print('<!DOCTYPE html>', file=docFD)
    print('<html lang="en">', file=docFD)
    print('<head>', file=docFD)
    print('  <META http-equiv="Content-Type" content="text/html; charset=UTF-8">', file=docFD)
    print('  <meta name="viewport" content="width=device-width, initial-scale=1.0" />', file=docFD)
    print('  <title>Documentation of the MBSim-Environment</title>', file=docFD)
    print('  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.6/css/bootstrap.min.css"/>', file=docFD)
    print('</head>', file=docFD)
    print('<body style="margin:1em">', file=docFD)
    print('<h1>Documentation of the MBSim-Environment</h1>', file=docFD)
    print('<div class="panel panel-success">', file=docFD)
    print('  <div class="panel-heading"><span class="glyphicon glyphicon-question-sign"></span>&nbsp;XML Documentation</div>', file=docFD)
    print('  <ul class="list-group">', file=docFD)
    print('    <li class="list-group-item"><a href="'+myurllib.pathname2url(pj("xmldoc", "http___mbsim_berlios_de_MBSimXML", "mbsimxml.html"))+'">MBSimXML</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</div>', file=docFD)
    print('<div class="panel panel-info">', file=docFD)
    print('  <div class="panel-heading"><span class="glyphicon glyphicon-question-sign"></span>&nbsp;Doxygen Documentation</div>', file=docFD)
    print('  <ul class="list-group">', file=docFD)
    for d in sorted(list(toolDoxyDocCopyDir)):
      print('    <li class="list-group-item"><a href="'+myurllib.pathname2url(pj("doc", d, "index.html"))+'">'+d+'</a></li>', file=docFD)
    print('  </ul>', file=docFD)
    print('</div>', file=docFD)
    print('<hr/>', file=docFD)
    print('<span class="pull-left small">', file=docFD)
    print('  <a href="/impressum_disclaimer_datenschutz.html#impressum">Impressum</a> /', file=docFD)
    print('  <a href="/impressum_disclaimer_datenschutz.html#disclaimer">Disclaimer</a> /', file=docFD)
    print('  <a href="/impressum_disclaimer_datenschutz.html#datenschutz">Datenschutz</a>', file=docFD)
    print('</span>', file=docFD)
    print('<span class="pull-right small">', file=docFD)
    print('  Generated on %s'%(str(timeID)), file=docFD)
    print('  <a href="http://validator.w3.org/check?uri=referer">', file=docFD)
    print('    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>', file=docFD)
    print('  </a>', file=docFD)
    print('</span>', file=docFD)
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

  # create index.html
  mainFD=codecs.open(pj(args.reportOutDir, "index.html"), "w", encoding="utf-8")
  print('''<!DOCTYPE html>
<html lang="en">
<head>
  <META http-equiv="Content-Type" content="text/html; charset=UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Build Results of MBSim-Env: <small>%s</small></title>
  <link rel="stylesheet" type="text/css" href="https://cdn.datatables.net/s/bs-3.3.5/jq-2.1.4,dt-1.10.10/datatables.min.css"/>
  <link rel="stylesheet" href="http://octicons.github.com/components/octicons/octicons/octicons.css"/>
</head>
<body style="margin:1em">
<script type="text/javascript" src="https://cdn.datatables.net/s/bs-3.3.5/jq-2.1.4,dt-1.10.10/datatables.min.js"> </script>
<script type="text/javascript" src="http://www.mbsim-env.de/mbsim/html/mbsimBuildServiceClient.js"></script>
<script type="text/javascript">
  $(document).ready(function() {
    $.fn.dataTableExt.sErrMode = 'throw';
    $('#SortThisTable').dataTable({'lengthMenu': [ [1, 5, 10, 25, -1], [1, 5, 10, 25, 'All'] ], 'pageLength': -1, 'aaSorting': [], stateSave: true});'''%(args.buildType), file=mainFD)

  if args.enableDistribution:
    releaseGeneration1(mainFD)

  print('''  } );
</script>

<h1>Build Results of MBSim-Env: <small>%s</small></h1>

<dl class="dl-horizontal">
<dt>Called Command</dt><dd><div class="dropdown">
  <button class="btn btn-default btn-xs" id="calledCommandID" data-toggle="dropdown">show <span class="caret"></span>
  </button>
  <code class="dropdown-menu" style="padding-left: 0.5em; padding-right: 0.5em;" aria-labelledby="calledCommandID">'''%(args.buildType), file=mainFD)
  for argv in sys.argv: print(argv.replace('/', u'/\u200B')+' ', file=mainFD)
  print('</code></div></dd>', file=mainFD)
  print('  <dt>Time ID</dt><dd>'+str(timeID)+'</dd>', file=mainFD)
  print('  <dt>End time</dt><dd><span id="STILLRUNNINGORABORTED" class="text-danger"><b>still running or aborted</b></span><!--E_ENDTIME--></dd>', file=mainFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  print('  <dt>Navigate</dt><dd><a class="btn btn-info btn-xs" href="../result_%010d/index.html"><span class="glyphicon glyphicon-step-backward"></span>&nbsp;previous</a>'%(currentID-1), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="../result_%010d/index.html"><span class="glyphicon glyphicon-step-forward"></span>&nbsp;next</a>'%(currentID+1), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="../result_current/index.html"><span class="glyphicon glyphicon-fast-forward"></span>&nbsp;newest</a>', file=mainFD)
  print('                    </dd>', file=mainFD)
  print('</dl>', file=mainFD)
  print('<hr/>', file=mainFD)

  nrFailed=0
  nrRun=0
  # update all repositories
  if not args.disableUpdate:
    nrRun+=1
  if repoUpdate(mainFD)!=0:
    nrFailed+=1

  # clean prefix dir
  if args.enableCleanPrefix and os.path.isdir(args.prefix if args.prefix!=None else args.prefixAuto):
    shutil.rmtree(args.prefix if args.prefix!=None else args.prefixAuto)
    os.makedirs(args.prefix if args.prefix!=None else args.prefixAuto)

  # force build
  buildTools=set()
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

  print('<h2>Build Status</h2>', file=mainFD)
  print('<p><span class="glyphicon glyphicon-info-sign"></span>&nbsp;Failures in the following table should be fixed from top to bottom since a error in one tool may cause errors on dependent tools.<br/>', file=mainFD)
  print('<span class="glyphicon glyphicon-info-sign"></span>&nbsp;A tool name in gray color is a tool which may fail and is therefore not reported as an error in the Atom feed.</p>', file=mainFD)

  print('<table id="SortThisTable" class="table table-striped table-hover table-bordered table-condensed">', file=mainFD)
  print('<thead><tr>', file=mainFD)
  print('<th><span class="glyphicon glyphicon-folder-open"></span>&nbsp;Tool</th>', file=mainFD)
  if not args.disableConfigure:
    print('<th><span class="glyphicon glyphicon-wrench"></span>&nbsp;Configure</th>', file=mainFD)
  if not args.disableMake:
    print('<th><span class="glyphicon glyphicon-repeat"></span>&nbsp;Make</th>', file=mainFD)
  if not args.disableMakeCheck:
    print('<th><span class="glyphicon glyphicon-ok-circle"></span>&nbsp;Check</th>', file=mainFD)
  if not args.disableDoxygen:
    print('<th><span class="glyphicon glyphicon-question-sign"></span>&nbsp;Doxygen Doc.</th>', file=mainFD)
  if not args.disableXMLDoc:
    print('<th><span class="glyphicon glyphicon-question-sign"></span>&nbsp;XML Doc.</th>', file=mainFD)
  print('</tr></thead><tbody>', file=mainFD)

  # list tools which are not updated and must not be rebuild according dependencies
  for tool in set(toolDependencies)-set(orderedBuildTools):
    print('<tr>', file=mainFD)
    print('<td>'+tool.replace('/', u'/\u200B')+'</td>', file=mainFD)
    for i in range(0, 5-sum([args.disableConfigure, args.disableMake, args.disableMakeCheck, args.disableDoxygen, args.disableXMLDoc])):
      print('<td>-</td>', file=mainFD)
    print('</tr>', file=mainFD)
  mainFD.flush()

  # build the other tools in order
  nr=1
  for tool in orderedBuildTools:
    nrFailedLocal, nrRunLocal=build(nr, len(orderedBuildTools), tool, mainFD)
    if toolDependencies[tool][0]==False:
      nrFailed+=nrFailedLocal
      nrRun+=nrRunLocal
    nr+=1

  # run examples
  runExamplesErrorCode=0
  if not args.disableRunExamples:
    savedDir=os.getcwd()
    os.chdir(pj(args.sourceDir, "mbsim", "examples"))
    print("Run runexamples.py in "+os.getcwd()); sys.stdout.flush()
    runExamplesErrorCode=runexamples(mainFD)
    os.chdir(savedDir)

  # create distribution
  if args.enableDistribution:
    nrRun=nrRun+1
    print("Create distribution"); sys.stdout.flush()
    cdRet, distArchiveName=createDistribution(mainFD)
    if cdRet!=0:
      nrFailed=nrFailed+1

  print('</tbody></table>', file=mainFD)

  if args.enableDistribution and nrFailed==0 and runExamplesErrorCode==0:
    releaseGeneration2(mainFD, distArchiveName)

  print('<hr/>', file=mainFD)
  print('<span class="pull-left small">', file=mainFD)
  print('  <a href="/impressum_disclaimer_datenschutz.html#impressum">Impressum</a> /', file=mainFD)
  print('  <a href="/impressum_disclaimer_datenschutz.html#disclaimer">Disclaimer</a> /', file=mainFD)
  print('  <a href="/impressum_disclaimer_datenschutz.html#datenschutz">Datenschutz</a>', file=mainFD)
  print('</span>', file=mainFD)
  print('<span class="pull-right small">', file=mainFD)
  print('  Generated on %s'%(str(timeID)), file=mainFD)
  print('  <a href="http://validator.w3.org/check?uri=referer">', file=mainFD)
  print('    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>', file=mainFD)
  print('  </a>', file=mainFD)
  print('</span>', file=mainFD)
  print('</body>', file=mainFD)
  print('</html>', file=mainFD)

  mainFD.close()
  # replace <span id="STILLRUNNINGORABORTED"...</span> in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    endTime=datetime.datetime.now()
    endTime=datetime.datetime(endTime.year, endTime.month, endTime.day, endTime.hour, endTime.minute, endTime.second)
    line=re.sub('<span id="STILLRUNNINGORABORTED".*?</span>', str(endTime), line)
    print(line, end="")

  # update build system state
  if args.buildSystemRun:
    buildSystemState.update(args.buildType+"-build", "Build Failed: "+args.buildType,
                            "%d of %d build parts failed."%(nrFailed, nrRun),
                            args.url+"/result_%010d"%(currentID)+"/index.html",
                            nrFailed, nrRun)

  if nrFailed>0:
    print("\nERROR: %d of %d build parts failed!!!!!"%(nrFailed, nrRun));

  return nrFailed+abs(runExamplesErrorCode)



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

def repoUpdate(mainFD):
  ret=0
  savedDir=os.getcwd()
  if not args.disableUpdate:
    print('Updating repositories: ', end="")

  print('<h2>Repository State</h2>', file=mainFD)
  print('<table style="width:auto;" class="table table-striped table-hover table-bordered table-condensed">', file=mainFD)
  print('<thead><tr>', file=mainFD)
  print('<th><span class="octicon octicon-repo"></span>&nbsp;Repository</th>', file=mainFD)
  print('<th><span class="octicon octicon-git-branch"></span>&nbsp;Branch</th>', file=mainFD)
  if not args.disableUpdate:
    print('<th><span class="glyphicon glyphicon-refresh"></span>&nbsp;Update</th>', file=mainFD)
  print('<th><span class="octicon octicon-git-commit"></span>&nbsp;Commit</th>', file=mainFD)
  print('</tr></thead><tbody>', file=mainFD)

  for repo in ["fmatvec", "hdf5serie", "openmbv", "mbsim"]:
    os.chdir(pj(args.sourceDir, repo+args.srcSuffix))
    # update
    repoUpdFD=codecs.open(pj(args.reportOutDir, "repo-update-"+repo+".txt"), "w", encoding="utf-8")
    retlocal=0
    if not args.disableUpdate:
      # write repUpd output to report dir
      print('Fetch remote repository '+repo+":", file=repoUpdFD)
      repoUpdFD.flush()
      retlocal+=abs(subprocess.check_call(["git", "fetch"], stdout=repoUpdFD, stderr=repoUpdFD))
    # set branch based on args
    if eval('args.'+repo+'Branch')!="":
      print('Checkout branch '+eval('args.'+repo+'Branch')+' in repository '+repo+":", file=repoUpdFD)
      retlocal+=abs(subprocess.check_call(["git", "checkout", eval('args.'+repo+'Branch')], stdout=repoUpdFD, stderr=repoUpdFD))
      repoUpdFD.flush()
    if not args.disableUpdate:
      print('Pull current branch', file=repoUpdFD)
      repoUpdFD.flush()
      retlocal+=abs(subprocess.check_call(["git", "pull"], stdout=repoUpdFD, stderr=repoUpdFD))
    # get branch and commit
    branch=subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD'], stderr=repoUpdFD).decode('utf-8').rstrip()
    commitid=subprocess.check_output(['git', 'log', '-n', '1', '--format=%h', 'HEAD'], stderr=repoUpdFD).decode('utf-8').rstrip()
    commitidfull=subprocess.check_output(['git', 'log', '-n', '1', '--format=%H', 'HEAD'], stderr=repoUpdFD).decode('utf-8').rstrip()
    commitsub=subprocess.check_output(['git', 'log', '-n', '1', '--format=%s', 'HEAD'], stderr=repoUpdFD).decode('utf-8').rstrip()
    commitshort="<code>"+commitid+"</code>: "+htmlEscape(commitsub)
    commitlong=subprocess.check_output(['git', 'log', '-n', '1', '--format=Commit: %H%nAuthor: %an%nDate:   %ad%n%s%n%b', 'HEAD'], stderr=repoUpdFD).decode('utf-8')
    commitlong=htmlEscape(commitlong)
    repoUpdFD.close()
    ret+=retlocal
    # output
    print('<tr>', file=mainFD)
    print('  <td><span class="label label-success"><span class="octicon octicon-repo"></span>&nbsp;'+repo+'</span></td>', file=mainFD)
    print('  <td><span class="label label-primary"><span class="octicon octicon-git-branch"></span>&nbsp;'+branch+'</span></td>', file=mainFD)
    if not args.disableUpdate:
      print('<td class="%s"><span class="glyphicon glyphicon-%s"></span>&nbsp;<a href="repo-update-%s.txt">%s</a></td>'%(
        "success" if retlocal==0 else "danger",
        "ok-sign alert-success" if retlocal==0 else "exclamation-sign alert-danger",
        repo,
        "passed" if retlocal==0 else "failed"), file=mainFD)
    print('  <td data-toggle="tooltip" data-placement="bottom" title="'+commitlong+'">'+commitshort+
          '<span id="COMMITID_%s" style="display:none">%s</span></td>'%(repo, commitidfull), file=mainFD)
    print('</tr>', file=mainFD)

  print('</tbody></table>', file=mainFD)
  mainFD.flush()

  if not args.disableUpdate:
    if ret>0:
      print('failed')
    else:
      print('passed')

  os.chdir(savedDir)
  return ret



def build(nr, nrAll, tool, mainFD):
  print("Building "+str(nr)+"/"+str(nrAll)+": "+tool+": ", end=""); sys.stdout.flush()

  nrFailed=0
  nrRun=0

  # start row, including tool name
  if toolDependencies[tool][0]==False:
    print('<tr>', file=mainFD)
  else:
    print('<tr class="text-muted">', file=mainFD)
  print('<td>'+tool.replace('/', u'/\u200B')+'</td>', file=mainFD)
  mainFD.flush()

  savedDir=os.getcwd()

  # configure
  print("configure", end=""); sys.stdout.flush()
  failed, run=configure(tool, mainFD)
  nrFailed+=failed
  nrRun+=run

  # cd to build dir
  os.chdir(savedDir)
  os.chdir(pj(args.sourceDir, buildTool(tool)))

  # make
  print(", make", end=""); sys.stdout.flush()
  failed, run=make(tool, mainFD)
  nrFailed+=failed
  nrRun+=run

  # make check
  print(", check", end=""); sys.stdout.flush()
  failed, run=check(tool, mainFD)
  nrFailed+=failed
  nrRun+=run

  # doxygen
  print(", doxygen-doc", end=""); sys.stdout.flush()
  failed, run=doc(tool, mainFD, args.disableDoxygen, "doc", toolDoxyDocCopyDir)
  nrFailed+=failed
  nrRun+=run

  # xmldoc
  print(", xml-doc", end=""); sys.stdout.flush()
  failed, run=doc(tool, mainFD, args.disableXMLDoc, "xmldoc", toolXMLDocCopyDir)
  nrFailed+=failed
  nrRun+=run

  os.chdir(savedDir)

  print("")
  print('</tr>', file=mainFD)
  mainFD.flush()

  return nrFailed, nrRun



def configure(tool, mainFD):
  if not os.path.isdir(pj(args.reportOutDir, tool)): os.makedirs(pj(args.reportOutDir, tool))
  configureFD=codecs.open(pj(args.reportOutDir, tool, "configure.txt"), "w", encoding="utf-8")
  copyConfigLog=False
  savedDir=os.getcwd()
  run=0
  try:
    if not args.disableConfigure:
      run=1
      # pre configure
      os.chdir(pj(args.sourceDir, srcTool(tool)))
      print("\n\nRUNNING aclocal\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["aclocal"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("aclocal failed")
      print("\n\nRUNNING autoheader\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["autoheader"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("autoheader failed")
      print("\n\nRUNNING libtoolize\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["libtoolize", "-c"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("libtoolize failed")
      print("\n\nRUNNING automake\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["automake", "-a", "-c"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("automake failed")
      print("\n\nRUNNING autoconf\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["autoconf"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("autoconf failed")
      print("\n\nRUNNING autoreconf\n", file=configureFD); configureFD.flush()
      if simplesandbox.call(["autoreconf"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
        raise RuntimeError("autoreconf failed")
      # configure
      os.chdir(savedDir)
      os.chdir(pj(args.sourceDir, buildTool(tool)))
      copyConfigLog=True
      print("\n\nRUNNING configure\n", file=configureFD); configureFD.flush()
      if args.prefix==None:
        if simplesandbox.call(["./config.status", "--recheck"], envvar=simplesandboxEnvvars, shareddir=["."],
                              stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
          raise RuntimeError("configure failed")
      else:
        command=[pj(args.sourceDir, srcTool(tool), "configure"), "--prefix", args.prefix]
        command.extend(args.passToConfigure)
        if simplesandbox.call(command, envvar=simplesandboxEnvvars, shareddir=["."],
                              stderr=subprocess.STDOUT, stdout=configureFD, buildSystemRun=args.buildSystemRun)!=0:
          raise RuntimeError("configure failed")
    else:
      print("configure disabled", file=configureFD); configureFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  if not args.disableConfigure:
    print('<td class="%s"><span class="glyphicon glyphicon-%s"></span>&nbsp;'%("success" if result=="done" else "danger",
      "ok-sign alert-success" if result=="done" else "exclamation-sign alert-danger"), file=mainFD)
    print('  <a href="'+myurllib.pathname2url(pj(tool, "configure.txt"))+'">'+result+'</a>', file=mainFD)
    if copyConfigLog:
      shutil.copyfile("config.log", pj(args.reportOutDir, tool, "config.log.txt"))
      print('  <a href="'+myurllib.pathname2url(pj(tool, "config.log.txt"))+'">config.log</a>', file=mainFD)
    print('</td>', file=mainFD)
  configureFD.close()
  mainFD.flush()
  os.chdir(savedDir)

  if result!="done":
    return 1, run
  return 0, run



def make(tool, mainFD):
  makeFD=codecs.open(pj(args.reportOutDir, tool, "make.txt"), "w", encoding="utf-8")
  run=0
  try:
    if not args.disableMake:
      run=1
      # make
      errStr=""
      if not args.disableMakeClean:
        print("\n\nRUNNING make clean\n", file=makeFD); makeFD.flush()
        if simplesandbox.call(["make", "clean"], envvar=simplesandboxEnvvars, shareddir=["."],
                              stderr=subprocess.STDOUT, stdout=makeFD, buildSystemRun=args.buildSystemRun)!=0:
          errStr=errStr+"make clean failed; "
      print("\n\nRUNNING make -k\n", file=makeFD); makeFD.flush()
      if simplesandbox.call(["make", "-k", "-j", str(args.j)], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=makeFD, buildSystemRun=args.buildSystemRun)!=0:
        errStr=errStr+"make failed; "
      if not args.disableMakeInstall:
        print("\n\nRUNNING make install\n", file=makeFD); makeFD.flush()
        if simplesandbox.call(["make", "-k", "install"], envvar=simplesandboxEnvvars,
                              shareddir=[".", args.prefix if args.prefix!=None else args.prefixAuto],
                              stderr=subprocess.STDOUT, stdout=makeFD, buildSystemRun=args.buildSystemRun)!=0:
          errStr=errStr+"make install failed; "
      if errStr!="": raise RuntimeError(errStr)
    else:
      print("make disabled", file=makeFD); makeFD.flush()

    result="done"
  except RuntimeError as ex:
    result=str(ex)
  if not args.disableMake:
    print('<td class="%s"><span class="glyphicon glyphicon-%s"></span>&nbsp;'%("success" if result=="done" else "danger",
      "ok-sign alert-success" if result=="done" else "exclamation-sign alert-danger"), file=mainFD)
    print('  <a href="'+myurllib.pathname2url(pj(tool, "make.txt"))+'">'+result+'</a>', file=mainFD)
    print('</td>', file=mainFD)
  makeFD.close()
  mainFD.flush()

  if result!="done":
    return 1, run
  return 0, run



def check(tool, mainFD):
  checkFD=codecs.open(pj(args.reportOutDir, tool, "check.txt"), "w", encoding="utf-8")
  run=0
  if not args.disableMakeCheck:
    run=1
    # make check
    print("RUNNING make check\n", file=checkFD); checkFD.flush()
    if simplesandbox.call(["make", "-j", str(args.j), "check"], envvar=simplesandboxEnvvars, shareddir=["."],
                          stderr=subprocess.STDOUT, stdout=checkFD, buildSystemRun=args.buildSystemRun)==0:
      result="done"
    else:
      result="failed"
  else:
    print("make check disabled", file=checkFD); checkFD.flush()
    result="done"

  foundTestSuiteLog=False
  testSuiteLogFD=codecs.open(pj(args.reportOutDir, tool, "test-suite.log.txt"), "w", encoding="utf-8")
  for rootDir,_,files in os.walk('.'): # append all test-suite.log files
    if "test-suite.log" in files:
      testSuiteLogFD.write('\n\n')
      testSuiteLogFD.write(open(pj(rootDir, "test-suite.log")).read())
      foundTestSuiteLog=True
  testSuiteLogFD.close()
  if not args.disableMakeCheck:
    print('<td class="%s"><span class="glyphicon glyphicon-%s"></span>&nbsp;'%("success" if result=="done" else "danger",
      "ok-sign alert-success" if result=="done" else "exclamation-sign alert-danger"), file=mainFD)
    print('  <a href="'+myurllib.pathname2url(pj(tool, "check.txt"))+'">'+result+'</a>', file=mainFD)
    if foundTestSuiteLog:
      print('  <a href="'+myurllib.pathname2url(pj(tool, "test-suite.log.txt"))+'">test-suite.log</a>', file=mainFD)
    print('</td>', file=mainFD)
  checkFD.close()
  mainFD.flush()

  if result!="done":
    return 1, run
  return 0, run



def doc(tool, mainFD, disabled, docDirName, toolDocCopyDir):
  if not os.path.isdir(docDirName):
    if docDirName=="doc" and not args.disableDoxygen or \
       docDirName=="xmldoc" and not args.disableXMLDoc:
      print('<td>not available</td>', file=mainFD)
    mainFD.flush()
    return 0, 0

  docFD=codecs.open(pj(args.reportOutDir, tool, docDirName+".txt"), "w", encoding="utf-8")
  savedDir=os.getcwd()
  os.chdir(docDirName)
  run=0
  try:
    if not disabled:
      run=1
      # make doc
      errStr=""
      print("\n\nRUNNING make clean\n", file=docFD); docFD.flush()
      if simplesandbox.call(["make", "clean"], envvar=simplesandboxEnvvars, shareddir=["."],
                            stderr=subprocess.STDOUT, stdout=docFD, buildSystemRun=args.buildSystemRun)!=0:
        errStr=errStr+"make clean failed; "
      print("\n\nRUNNING make\n", file=docFD); docFD.flush()
      if simplesandbox.call(["make", "-k"], envvar=simplesandboxEnvvars,
                            shareddir=[".", args.prefix if args.prefix!=None else args.prefixAuto],
                            stderr=subprocess.STDOUT, stdout=docFD, buildSystemRun=args.buildSystemRun)!=0:
        errStr=errStr+"make failed; "
      print("\n\nRUNNING make install\n", file=docFD); docFD.flush()
      if simplesandbox.call(["make", "-k", "install"], envvar=simplesandboxEnvvars,
                            shareddir=[".", args.prefix if args.prefix!=None else args.prefixAuto],
                            stderr=subprocess.STDOUT, stdout=docFD, buildSystemRun=args.buildSystemRun)!=0:
        errStr=errStr+"make install failed; "
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
  if docDirName=="doc" and not args.disableDoxygen or \
     docDirName=="xmldoc" and not args.disableXMLDoc:
    print('<td class="%s"><span class="glyphicon glyphicon-%s"></span>&nbsp;'%("success" if result=="done" else "danger",
      "ok-sign alert-success" if result=="done" else "exclamation-sign alert-danger"), file=mainFD)
    print('  <a href="'+myurllib.pathname2url(pj(tool, docDirName+".txt"))+'">'+result+'</a>', file=mainFD)
    print('</td>', file=mainFD)
  docFD.close()
  mainFD.flush()

  if result!="done":
    return 1, run
  return 0, run



def runexamples(mainFD):
  if args.disableRunExamples:
    mainFD.flush()
    return 0

  print('<tr><td>Run examples</td>', file=mainFD); mainFD.flush()

  # runexamples.py command
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  command=["./runexamples.py", "-j", str(args.j)]
  if args.url!=None:
    command.extend(["--url", args.url+"/result_%010d/runexamples_report"%(currentID)])
  command.extend(["--buildType", args.buildType])
  command.extend(["--reportOutDir", pj(args.reportOutDir, "runexamples_report")])
  command.extend(["--currentID", str(currentID)])
  command.extend(["--timeID", timeID.strftime("%Y-%m-%dT%H:%M:%S")])
  if args.buildSystemRun:
    command.extend(["--buildSystemRun", scriptdir])
  command.extend(args.passToRunexamples)

  print("")
  print("")
  print("Output of runexamples.py")
  print("")
  if not os.path.isdir(pj(args.reportOutDir, "runexamples_report")): os.makedirs(pj(args.reportOutDir, "runexamples_report"))
  ret=abs(simplesandbox.call(command, envvar=simplesandboxEnvvars, shareddir=[".", pj(args.reportOutDir, "runexamples_report"),
                             "/var/www/html/mbsim/buildsystemstate"],
                             stderr=subprocess.STDOUT, buildSystemRun=args.buildSystemRun))

  if ret==0:
    print('<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;<a href="'+myurllib.pathname2url(pj("runexamples_report", "result_current", "index.html"))+
      '">all examples passed</a></td>', file=mainFD)
  else:
    print('<td class="danger"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;<a href="'+myurllib.pathname2url(pj("runexamples_report", "result_current", "index.html"))+
      '">examples failed</a></td>', file=mainFD)
  for i in range(0, 4-sum([args.disableConfigure, args.disableMake, args.disableMakeCheck, args.disableDoxygen, args.disableXMLDoc])):
    print('<td>-</td>', file=mainFD)
  print('</tr>', file=mainFD)

  mainFD.flush()

  return ret



def createDistribution(mainFD):
  print('<tr><td>Create distribution</td>', file=mainFD); mainFD.flush()
  os.mkdir(pj(args.reportOutDir, "distribute"))
  distLog=codecs.open(pj(args.reportOutDir, "distribute", "log.txt"), "w", encoding="utf-8")
  distArchiveName="failed"
  distributeErrorCode=simplesandbox.call([pj(scriptdir, "distribute.py"), "--outDir", pj(args.reportOutDir, "distribute"),
                                         args.prefix if args.prefix!=None else args.prefixAuto],
                                         buildSystemRun=args.buildSystemRun, shareddir=[pj(args.reportOutDir, "distribute")],
                                         stderr=subprocess.STDOUT, stdout=distLog)
  distLog.close()
  if distributeErrorCode==0:
    lines=codecs.open(pj(args.reportOutDir, "distribute", "log.txt"), "r", encoding="utf-8").readlines()
    distArchiveName=[x[len("distArchiveName="):] for x in lines if x.startswith("distArchiveName=")][0].rstrip()
    debugArchiveName=[x[len("debugArchiveName="):] for x in lines if x.startswith("debugArchiveName=")][0].rstrip()
    print('<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;'+
          '<a href="'+myurllib.pathname2url(pj("distribute", "log.txt"))+'">done</a> - '+
          '<a href="'+myurllib.pathname2url(pj("distribute", distArchiveName))+'"><b>Download</b></a> - '+
          '<a href="'+myurllib.pathname2url(pj("distribute", debugArchiveName))+'">Debug-Info</a>'+
          '</td>', file=mainFD)
  else:
    print('<td class="danger"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;'+
          '<a href="'+myurllib.pathname2url(pj("distribute", "log.txt"))+'">failed</a>'+
          '</td>', file=mainFD)
  for i in range(0, 4-sum([args.disableConfigure, args.disableMake, args.disableMakeCheck, args.disableDoxygen, args.disableXMLDoc])):
    print('<td>-</td>', file=mainFD)
  print('</tr>', file=mainFD); mainFD.flush()

  return distributeErrorCode, distArchiveName



def releaseGeneration1(mainFD):
  print('''    // no initial communication needed -> set OK status
    statusMessage({success: true, message: "ready"}); // no initial communication needed -> set OK status
    // when a release version is entered update the button text
    $("#RELEASEVERSION").keyup(function() {
      curRelStr=$("#RELEASEVERSION").val();
      $(".RELSTR").each(function() {
        $(this).text(curRelStr);
      })
    });
    // when the release button is clicked
    $("#RELEASEBUTTON").click(function() {
      // check if all checkboxes are checked
      checkBoxUnchecked=false;
      $(".RELEASECHECK").each(function() {
        if(!$(this).prop("checked"))
          checkBoxUnchecked=true;
      });
      // get data
      var data={login: localStorage['GITHUB_LOGIN_NAME'], athmac: localStorage['GITHUB_LOGIN_ATHMAC'],
                distArchiveName: $("#DISTARCHIVENAME").text(),
                reportOutDir: $("#REPORTOUTDIR").text(),
                relStr: $("#RELEASEVERSION").val(),
                commitid: {fmatvec:   $("#COMMITID_fmatvec").text(),
                           hdf5serie: $("#COMMITID_hdf5serie").text(),
                           openmbv:   $("#COMMITID_openmbv").text(),
                           mbsim:     $("#COMMITID_mbsim").text()}};
      if(checkBoxUnchecked || data.relStr=="")
        statusMessage({success: false, message: "You must first check all checklist items above and define the release string!"});
      else {
        statusCommunicating();
        // send data to server
        $.ajax({url: cgiPath+"/releasedistribution",
                dataType: "json", type: "POST", data: JSON.stringify(data)
              }).done(function(response) {
          statusMessage(response);
        });
      }
    });''', file=mainFD)

def releaseGeneration2(mainFD, distArchiveName):
  # default values
  relStr="x.y"
  relArchiveNamePrefix=re.sub("(.*-)xxx\..*", "\\1",  distArchiveName)
  relArchiveNamePostfix=re.sub(".*-xxx(\..*)", "\\1",  distArchiveName)
  tagNamePrefix="release/"
  tagNamePostfix=re.sub("mbsim-env-(.*)-shared-build-xxx.*", "-\\1", distArchiveName)

  print('''<div class="panel panel-warning">
  <div class="panel-heading"><span class="glyphicon glyphicon-pencil">
    </span>&nbsp;<a data-toggle="collapse" href="#collapseReleaseGeneration">
 Release this distribution<span class="caret"> </span></a></div>
  <div class="panel-body panel-collapse collapse" id="collapseReleaseGeneration">
    <p>Releasing this distribution will</p>
    <ul>
      <li>tag the commits of the repositories, shown at the top, on GitHub.</li>
      <li>copy the above distribution (<b>Download</b> - Debug-Info) to the <a href="../../../releases">release directory</a>.</li>
    </ul>
    <p>When releasing a distribution you have</p>
    <div style="margin-left:1.5em">
      <div class="checkbox"><label>
        <input type="checkbox" class="RELEASECHECK"/>
        first to check that the corresponding debug build works including all examples.
      </label></div>
      <div class="checkbox"><label>
        <input type="checkbox" class="RELEASECHECK"/>
        first to check that the corresponging valgrind-examples of the debug build works.
      </label></div>
      <div class="checkbox"><label>
        <input type="checkbox" class="RELEASECHECK"/>
        first to download the distribution and check it manually on a native OS (at least
        using the test script .../mbsim-env/bin/mbsim-env-test[.bat]).
      </label></div>
      <div class="checkbox"><label>
        <input type="checkbox" class="RELEASECHECK"/>
        to release the Windows and Linux release builds at the same commit state using the same "release version" string!
      </label></div>
    </div>
    <p><small>(This server stores your username and an application specific private GitHub access token. Logout removes both data. You can also revoke this token on GitHub at any time to revoke any access of this server on your GitHub account. Your GitHub password is not known by this server but checked by GitHub on login.)</small></p>
    <div>
      <span class="octicon octicon-person"></span>&nbsp;
      <strong id="LOGINUSER">unknwon</strong>
      <button id="LOGINBUTTON" type="button" disabled="disabled" class="btn btn-default btn-sm"><span class="octicon octicon-sign-in">
        </span>&nbsp;Login using <span class="octicon octicon-logo-github"></span></button>
      <button id="LOGOUTBUTTON" type="button" disabled="disabled" class="btn btn-default btn-sm"><span class="octicon octicon-sign-out"></span>&nbsp;Logout</button>
    </div>
    <div>
      <span id="DISTARCHIVENAME" style="display:none">%s</span>
      <span id="REPORTOUTDIR" style="display:none">%s</span>
      <div>
        <label for="RELEASEVERSION">Release version: </label>
        <input type="text" class="form-control" id="RELEASEVERSION" placeholder="%s">
      </div>
    </div>
    <div>
      <button id="RELEASEBUTTON" type="button" disabled="disabled" class="btn btn-default"><span class="glyphicon glyphicon-cloud-upload"></span>&nbsp;Release as <b>%s<span class="RELSTR">%s</span>%s</b> and tag as <b>%s<span class="RELSTR">%s</span>%s</b></button>
    </div>
    <p><small>(This will create an annotated git tag on the MBSim-Env repositories on GitHub with your GitHub account.)</small></p>
  </div>
</div>
<div id="STATUSPANEL" class="panel panel-info">
  <div class="panel-heading"><span class="glyphicon glyphicon-info-sign">
    </span>&nbsp;<span class="glyphicon glyphicon-exclamation-sign"></span>&nbsp;Status message</div>
  <div class="panel-body">
    <span id="STATUSMSG">Communicating with server, please wait. (reload page if hanging)</span>
  </div>
</div>'''%(distArchiveName, args.reportOutDir, relStr, relArchiveNamePrefix, relStr, relArchiveNamePostfix,
           tagNamePrefix, relStr, tagNamePostfix), file=mainFD)



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)

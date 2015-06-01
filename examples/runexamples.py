#! /usr/bin/python

# imports
from __future__ import print_function # to enable the print function for backward compatiblity with python2
import sys
if sys.version_info[0]==2 and sys.version_info[1]<=6 or sys.version_info[0]==3 and sys.version_info[1]<=1 :
  print("At least python version 2 >= 2.7 or python version 3 >= 3.2 is required!")
  exit(1)
import argparse
import fnmatch
import os
import stat
from os.path import join as pj
import subprocess
import datetime
import fileinput
import glob
import shutil
import functools
import multiprocessing
import math
import traceback
import tarfile
import re
import hashlib
import codecs
import threading
import time
import json
import fcntl
if sys.version_info[0]==2: # to unify python 2 and python 3
  import urllib as myurllib
else:
  import urllib.request as myurllib

# global variables
mbsimBinDir=None
canCompare=True # True if numpy and h5py are found
mbxmlutilsvalidate=None
ombvSchema =None
mbsimXMLSchema=None
timeID=None
directories=list() # a list of all examples sorted in descending order (filled recursively (using the filter) by by --directories)
# the following examples will fail: do not report them in the RSS feed as errors
willFail=set([
  pj("mechanics", "flexible_body", "rotor"),
  pj("mechanics", "flexible_body", "beltdrive"),
  pj("mechanics", "contacts", "self_siphoning_beads"),
  pj("mechanics", "flexible_body", "spatial_beam_cosserat"),
  pj("mechanics", "flexible_body", "pearlchain_cosserat_2D_POD"),
  pj("mechanics", "flexible_body", "block_compression"),
  pj("mechanics", "flexible_body", "linear_external_ffr_spatial_beam"),
  pj("mechanics", "flexible_body", "flexible_crank_mechanism_fe"),
  pj("mechanics", "contacts", "point_nurbsdisk"),
  pj("mechanics", "contacts", "circle_nurbsdisk2s"),
  pj("mechanics", "basics", "slider_crank"),
  pj("mechanics", "flexible_body", "mfr_mindlin"),
  pj("fmi", "hierachical_modelling"),
  pj("fmi", "simple_test"),
  pj("fmi", "sphere_on_plane"),
])

# MBSim Modules
mbsimModules=["mbsimControl", "mbsimElectronics", "mbsimFlexibleBody",
              "mbsimHydraulics", "mbsimInterface", "mbsimPowertrain"]

# command line option definition
argparser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description='''
  Run MBSim examples.
  This script runs the action given by --action on all specified directories recursively.
  However only examples of the type matching --filter are executed.
  The specified directories are processed from left to right.
  The type of an example is defined dependent on some key files in the corrosponding example directory:
  - If a file named 'Makefile' exists, than it is treated as a SRC example.
  - If a file named 'MBS.mbsimprj.flat.xml' exists, then it is treated as a FLATXML example.
  - If a file named 'MBS.mbsimprj.xml' exists, then it is treated as a XML example which run throught the MBXMLUtils preprocessor first.
  - If a file named 'FMI.mbsimprj.xml' exists, then it is treated as a FMI XML export example.
    Beside running the file by mbsimxml also mbsimCreateFMU is run to export the model as a FMU and the FMU is run by fmuCheck.<PLATFORM>.
  - If a file named 'Makefile_FMI' exists, then it is treated as a FMI source export example.
    Beside compiling the source examples also mbsimCreateFMU is run to export the model as a FMU and the FMU is run by fmuCheck.<PLATFORM>.
  If more then one of these files exist the behaviour is undefined.
  The 'Makefile' of a SRC example must build the example and must create an executable named 'main'.
  '''
)

mainOpts=argparser.add_argument_group('Main Options')
mainOpts.add_argument("directories", nargs="*", default=os.curdir,
  help='''A directory to run (recursively). If prefixed with '^' remove the directory form the current list
          If starting with '@' read directories from the file after the '@'. This file must provide
          the directories as a list of strings under the "checkedExamples" name in JSON format. Each directory
          in the file may itself be prefixed with ^ or @.
          Note that the directories in the file (JSON name "checkedExamples") is cleared, hence the file is modified.''')
mainOpts.add_argument("--action", default="report", type=str,
  help='''The action of this script:
          'report': run examples and report results (default);
          'copyToReference': copy current results to reference directory;
          'updateReference[=URL|DIR]': update references from URL or DIR, use the build system if not given;
          'pushReference=DIR': push references to DIR;
          'list': list directories to be run;''')
mainOpts.add_argument("-j", default=1, type=int, help="Number of jobs to run in parallel (applies only to the action 'report')")
mainOpts.add_argument("--filter", default="True", type=str,
  help='''Filter the specifed directories using the given Python code. A directory is processed if the provided
          Python code evaluates to True where the following variables are defined:
          src: is True if the directory is a source code example;
          flatxml: is True if the directory is a xml flat example;
          ppxml: is True if the directory is a preprocessing xml example;
          xml: is True if the directory is a flat or preprocessing xml example;
          fmi: is True if the directory is a FMI export example (source or XML);
          mbsimXXX: is True if the example in the directory uses the MBSim XXX module.
                    mbsimXXX='''+str(mbsimModules)+''';
          Example: --filter "xml and not mbsimControl": run xml examples not requiring mbsimControl''')

cfgOpts=argparser.add_argument_group('Configuration Options')
cfgOpts.add_argument("--atol", default=2e-5, type=float,
  help="Absolute tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--rtol", default=2e-5, type=float,
  help="Relative tolerance. Channel comparing failed if for at least ONE datapoint the abs. AND rel. toleranz is violated")
cfgOpts.add_argument("--disableRun", action="store_true", help="disable running the example on action 'report'")
cfgOpts.add_argument("--disableMakeClean", action="store_true", help="disable make clean on action 'report'")
cfgOpts.add_argument("--disableCompare", action="store_true", help="disable comparing the results on action 'report'")
cfgOpts.add_argument("--disableValidate", action="store_true", help="disable validating the XML files on action 'report'")
cfgOpts.add_argument("--printToConsole", action='store_const', const=sys.stdout, help="print all output also to the console")
cfgOpts.add_argument("--buildType", default="", type=str, help="Description of the build type (e.g: 'Daily Build: ')")
cfgOpts.add_argument("--prefixSimulation", default=None, type=str,
  help="prefix the simulation command (./main, mbsimflatxml, mbsimxml) with this string: e.g. 'valgrind --tool=callgrind'")
cfgOpts.add_argument("--prefixSimulationKeyword", default=None, type=str,
  help="VALGRIND: add special arguments and handling for valgrind")
cfgOpts.add_argument("--exeExt", default="", type=str, help="File extension of cross compiled executables")
cfgOpts.add_argument("--maxExecutionTime", default=30, type=float, help="The time in minutes after started program timed out")

outOpts=argparser.add_argument_group('Output Options')
outOpts.add_argument("--reportOutDir", default="runexamples_report", type=str, help="the output directory of the report")
outOpts.add_argument("--url", type=str,
  help="the URL where the report output is accessible (without the trailing '/index.html'. Only used for the RSS feed")
outOpts.add_argument("--rotate", default=3, type=int, help="keep last n results and rotate them")

debugOpts=argparser.add_argument_group('Debugging and other Options')
debugOpts.add_argument("--debugDisableMultiprocessing", action="store_true",
  help="disable the -j option and run always in a single process/thread")
debugOpts.add_argument("--currentID", default=0, type=int, help="Internal option used in combination with build.py")
debugOpts.add_argument("--timeID", default="", type=str, help="Internal option used in combination with build.py")

# parse command line options
args = argparser.parse_args()

# A file object which prints to multiple files
class MultiFile(object):
  def __init__(self, file1, second=None):
    self.filelist=[file1]
    if second!=None:
      self.filelist.append(second)
  def write(self, str):
    for f in self.filelist:
      f.write(str)
  def flush(self):
    for f in self.filelist:
      f.flush()
  def close(self):
    for f in self.filelist:
      if f!=sys.stdout and f!=sys.stderr:
        f.close()
# kill the called subprocess
def killSubprocessCall(proc, f, killed):
  killed.set()
  f.write("\n\n\n******************** START: MESSAGE FROM runexamples.py ********************\n")
  f.write("The maximal execution time (%d min) has reached (option --maxExecutionTime),\n"%(args.maxExecutionTime))
  f.write("but the program is still running. Terminating the program now.\n")
  f.write("******************** END: MESSAGE FROM runexamples.py **********************\n\n\n\n")
  proc.terminate()
  time.sleep(30)
  # if proc has not terminated after 30 seconds kill it
  if proc.poll()==None:
    f.write("\n\n\n******************** START: MESSAGE FROM runexamples.py ********************\n")
    f.write("Program has not terminated after 30 seconds, killing the program now.\n")
    f.write("******************** END: MESSAGE FROM runexamples.py **********************\n\n\n\n")
    proc.kill()
# subprocess call with MultiFile output
def subprocessCall(args, f, env=os.environ, maxExecutionTime=0):
  # start the program to execute
  proc=subprocess.Popen(args, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, bufsize=-1, env=env)
  # a guard for the maximal execution time for the starte program
  guard=None
  killed=threading.Event()
  if maxExecutionTime>0:
    guard=threading.Timer(maxExecutionTime*60, killSubprocessCall, args=(proc, f, killed))
    guard.start()
  # read all output in 100 byte blocks
  lineNP=b'' # not already processed bytes (required since we read 100 bytes which may break a unicode multi byte character)
  while True:
    line=lineNP+proc.stdout.read(100)
    lineNP=b''
    if line==b'': break
    try:
      print(line.decode("utf-8"), end="", file=f)
    except UnicodeDecodeError as ex: # catch broken multibyte unicode characters and append it to next line
      print(line[0:ex.start].decode("utf-8"), end="", file=f) # print up to first broken character
      lineNP=ex.object[ex.start:] # add broken characters to next line
  # wait for the call program to exit
  ret=proc.wait()
  # stop the execution time guard thread
  if maxExecutionTime>0:
    if killed.isSet():
      return None # return None to indicate that the program was terminated/killed
    else:
      guard.cancel()
  # return the return value ot the called programm
  return ret

# rotate
def rotateOutput():
  # create output dir
  if not os.path.isdir(args.reportOutDir): os.makedirs(args.reportOutDir)

  if args.currentID==0:
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
  else:
    currentID=args.currentID
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
  # check arguments
  if not (args.action=="report" or args.action=="copyToReference" or
          args.action=="updateReference" or args.action.startswith("updateReference=") or
          args.action.startswith("pushReference=") or
          args.action=="list"):
    argparser.print_usage()
    print("error: unknown argument --action "+args.action+" (see -h)")
    return 1
  args.updateURL="http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/references" # default value
  args.pushDIR=None # no default value (use /var/www/html/mbsim-env/MBSimDailyBuild/references for the build system)
  if args.action.startswith("updateReference="):
    if os.path.isdir(args.action[16:]):
      args.updateURL="file://"+myurllib.pathname2url(os.path.abspath(args.action[16:]))
    else:
      args.updateURL=args.action[16:]
    args.action="updateReference"
  if args.action.startswith("pushReference="):
    args.pushDIR=args.action[14:]
    args.action="pushReference"

  # fix arguments
  args.reportOutDir=os.path.abspath(args.reportOutDir)
  if args.prefixSimulation!=None:
    args.prefixSimulation=args.prefixSimulation.split(' ')
  else:
    args.prefixSimulation=[]
  global scriptDir
  scriptDir=os.path.dirname(os.path.realpath(__file__))

  # rotate (modifies args.reportOutDir)
  rotateOutput()
  os.makedirs(pj(args.reportOutDir, "tmp"))

  # check if the numpy and h5py modules exists. If not disable compare
  try: 
    import numpy
    import h5py
  except ImportError: 
    print("WARNING!")
    print("The python module numpy and h5py is required for full functionallity of this script.")
    print("However at least one of these modules are not found. Hence comparing the results will be disabled.\n")
    global canCompare
    canCompare=False
  # get mbxmlutilsvalidate program
  global mbxmlutilsvalidate
  mbxmlutilsvalidate=pj(pkgconfig("mbxmlutils", ["--variable=BINDIR"]), "mbxmlutilsvalidate"+args.exeExt)
  if not os.path.isfile(mbxmlutilsvalidate):
    mbxmlutilsvalidate="mbxmlutilsvalidate"+args.exeExt
  # set global dirs
  global mbsimBinDir
  mbsimBinDir=pkgconfig("mbsim", ["--variable=bindir"])
  # get schema files
  schemaDir=pkgconfig("mbxmlutils", ["--variable=SCHEMADIR"])
  global ombvSchema, mbsimXMLSchema
  ombvSchema =pj(schemaDir, "http___openmbv_berlios_de_OpenMBV", "openmbv.xsd")
  # create mbsimxml schema
  mbsimXMLSchema=pj(args.reportOutDir, "tmp", "mbsimxml.xsd") # generated it here
  subprocess.check_call([pj(mbsimBinDir, "mbsimxml"+args.exeExt), "--onlyGenerateSchema", mbsimXMLSchema])

  # if no directory is specified use the current dir (all examples) filter by --filter
  if len(args.directories)==0:
    dirs=[os.curdir]
  else:
    dirs=args.directories
  # loop over all directories on command line and add subdir which match the filter
  directoriesSet=set()
  for d in dirs:
    addExamplesByFilter(d, directoriesSet)

  # sort directories in descending order of simulation time (reference time)
  sortDirectories(directoriesSet, directories)

  # copy the current solution to the reference directory
  if args.action=="copyToReference":
    copyToReference()
    return 0

  # apply (unpack) a reference archive
  if args.action=="pushReference":
    pushReference()
    return 0

  # apply (unpack) a reference archive
  if args.action=="updateReference":
    updateReference()
    return 0

  # list directires to run
  if args.action=="list":
    listExamples()
    return 0

  # write empty RSS feed
  writeRSSFeed(0, 1) # nrFailed == 0 => write empty RSS feed
  # create index.html
  mainFD=codecs.open(pj(args.reportOutDir, "index.html"), "w", encoding="utf-8")
  print('''<!DOCTYPE html>
  <html lang="en">
  <head>
    <META http-equiv="Content-Type" content="text/html; charset=UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>MBSim runexamples Results</title>
    <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>
    <link rel="stylesheet" href="http://octicons.github.com/components/octicons/octicons/octicons.css"/>
    <link rel="stylesheet" href="http://cdn.datatables.net/1.10.2/css/jquery.dataTables.css"/>
    <link rel="alternate" type="application/rss+xml" title="MBSim runexample.py Result" href="../result.rss.xml"/>
  </head>
  <body style="margin:1em">
  <script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>
  <script type="text/javascript" src="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/js/bootstrap.min.js"> </script>
  <script type="text/javascript" src="http://cdn.datatables.net/1.10.2/js/jquery.dataTables.min.js"> </script>
  <script type="text/javascript" src="http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/mbsimBuildServiceClient.js"></script>
  <script type="text/javascript">
    $(document).ready(function() {
      // init table
      $('#SortThisTable').dataTable({'lengthMenu': [ [10, 25, 50, 100, -1], [10, 25, 50, 100, 'All'] ],
                                     'pageLength': 25, 'aaSorting': [], stateSave: true});

      // when the save button is clicked
      $("#SAVEBUTTON").click(function() {
        statusCommunicating();
        // collect all data
        var checkedExamples=[];
        $("#SortThisTable").DataTable().$("._EXAMPLE").each(function() {
          if($(this).prop("checked")) {
            checkedExamples.push($(this).attr("name"));
          }
        });
        // save current checked examples
        var data={login: localStorage['GITHUB_LOGIN_NAME'], athmac: localStorage['GITHUB_LOGIN_ATHMAC'], checkedExamples: checkedExamples}
        $.ajax({url: cgiPath+"/setcheck",
                dataType: "json", type: "POST", data: JSON.stringify(data)
              }).done(function(response) {
          statusMessage(response);
        });
      });

      // if this is the current example table from the build server and is finished than enable the reference update
      if(($(location).attr('href')=="http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/result_current/runexamples_report/result_current/" ||
          $(location).attr('href')=="http://www4.amm.mw.tu-muenchen.de:8080/mbsim-env/MBSimDailyBuild/report/result_current/runexamples_report/result_current/index.html") &&
          $("#FINISHED").length>0) {
        // show reference update and status
        $("#UPDATEREFERENCES").css("display", "block");
        $("#STATUSPANEL").css("display", "block");

        // update checked examples using server data
        statusCommunicating();
        $.ajax({url: cgiPath+"/getcheck",
                dataType: "json", type: "POST",
               }).done(function(response) {
          if(!response.success)
            statusMessage(response);
          else {
            // "check" and enable these
            $("#SortThisTable").DataTable().$("._EXAMPLE").each(function() {
              $(this).prop("checked", $.inArray($(this).attr("name"), response.checkedExamples)>=0);
              $(this).prop("disabled", false);
            });
            statusMessage(response);
          }
        });
      }
    });
  </script>''', file=mainFD)

  print('<h1>MBSim runexamples Results</h1>', file=mainFD)
  print('<dl class="dl-horizontal">', file=mainFD)
  print('  <dt>Called command</dt><dd><code>', file=mainFD)
  for argv in sys.argv: print(argv.replace('/', u'/\u200B')+' ', file=mainFD)
  print('  </code></dd>', file=mainFD)
  print('  <dt>RSS Feed</dt><dd><span class="octicon octicon-rss"></span>&nbsp;Use the feed "auto-discovery" of this page or click <a href="../result.rss.xml">here</a></dd>', file=mainFD)
  global timeID
  timeID=datetime.datetime.now()
  timeID=datetime.datetime(timeID.year, timeID.month, timeID.day, timeID.hour, timeID.minute, timeID.second)
  if args.timeID!="":
    timeID=datetime.datetime.strptime(args.timeID, "%Y-%m-%dT%H:%M:%S")
  print('  <dt>Time ID</dt><dd>'+str(timeID)+'</dd>', file=mainFD)
  print('  <dt>End time</dt><dd><!--S_ENDTIME--><span class="text-danger"><b>still running or aborted</b></span><!--E_ENDTIME--></dd>', file=mainFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('  <dt>Navigate</dt><dd><a class="btn btn-info btn-xs" href="..%s/result_%010d%s/index.html"><span class="glyphicon glyphicon-step-backward"> </span> previous</a>'%(navA, currentID-1, navB), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="..%s/result_%010d%s/index.html"><span class="glyphicon glyphicon-step-forward"> </span> next</a>'%(navA, currentID+1, navB), file=mainFD)
  print('                    <a class="btn btn-info btn-xs" href="..%s/result_current%s/index.html"><span class="glyphicon glyphicon-fast-forward"> </span> newest</a>'%(navA, navB), file=mainFD)
  if args.currentID!=0:
    print('                  <a class="btn btn-info btn-xs" href="../../index.html"><span class="glyphicon glyphicon-eject"> </span> parent</a>', file=mainFD)
  print('                    </dd>', file=mainFD)
  print('</dl>', file=mainFD)
  print('<hr/><p><span class="glyphicon glyphicon-info-sign"> </span> A example with grey text is a example which may fail and is therefore not reported as an error in the RSS feed.</p>', file=mainFD)

  print('<table id="SortThisTable" class="table table-striped table-hover table-bordered table-condensed">', file=mainFD)
  print('<thead><tr>', file=mainFD)
  print('<th><span class="glyphicon glyphicon-folder-open"></span>&nbsp;Example</th>', file=mainFD)
  if not args.disableRun:
    print('<th><span class="glyphicon glyphicon-repeat"></span>&nbsp;Run</th>', file=mainFD)
    print('<th><span class="glyphicon glyphicon-time"></span>&nbsp;Time</th>', file=mainFD)
    print('<th><span class="glyphicon glyphicon-time"></span>&nbsp;Ref. Time</th>', file=mainFD)
  if not args.disableCompare:
    print('<th><div class="pull-left"><span class="glyphicon glyphicon-search"></span>&nbsp;Ref.</div>'+\
          '<div class="pull-right" style="padding-right:0.75em;">[update]</div></th>', file=mainFD)
  if not args.disableRun:
    print('<th><span class="glyphicon glyphicon-warning-sign"></span>&nbsp;Depr.</th>', file=mainFD)
  if not args.disableValidate:
    print('<th><span class="glyphicon glyphicon-ok"></span>&nbsp;XML out.</th>', file=mainFD)
  print('</tr></thead><tbody>', file=mainFD)
  mainFD.flush()
  mainRet=0
  failedExamples=[]

  # run examples in parallel
  print("Started running examples. Each example will print a message if finished.")
  print("See the log file "+pj(os.path.dirname(args.reportOutDir), "result_current", "index.html")+" for detailed results.\n")

  if not args.debugDisableMultiprocessing:
    # init mulitprocessing handling and run in parallel
    resultQueue = multiprocessing.Manager().Queue()
    poolResult=multiprocessing.Pool(args.j).map_async(functools.partial(runExample, resultQueue), directories, 1)
  else: # debugging
    if sys.version_info[0]==2: # to enable backward compatiblity with python2
      from Queue import Queue as MyQueue
    else:
      from queue import Queue as MyQueue
    resultQueue = MyQueue()
    poolResult=MyQueue(); poolResult.put(list(map(functools.partial(runExample, resultQueue), directories)))

  # get the queue values = html table row
  missingDirectories=list(directories)
  for dummy in directories:
    result=resultQueue.get()
    print(result[1], file=mainFD)
    mainFD.flush()
    printFinishedMessage(missingDirectories, result)
  # wait for pool to finish and get result
  retAll=poolResult.get()
  # set global result and add failedExamples
  for index in range(len(retAll)):
    if retAll[index]!=0 and not directories[index][0] in willFail:
      mainRet=1
      failedExamples.append(directories[index][0])

  print('</tbody></table><hr/>', file=mainFD)

  if len(failedExamples)>0:
    print('<div class="panel panel-info">', file=mainFD)
    print('  <div class="panel-heading"><span class="glyphicon glyphicon-refresh"></span>&nbsp;<a data-toggle="collapse" href="#collapseRerunFailedExamples">'+\
            'Rerun all failed examples<span class="caret"> </span>'+\
            '</a></div>', file=mainFD)
    print('  <div class="panel-body panel-collapse collapse" id="collapseRerunFailedExamples">', file=mainFD)
    print('<code>'+sys.argv[0], end=" ", file=mainFD)
    for arg in sys.argv[1:]:
      if not arg in set(args.directories):
        print(arg, end=" ", file=mainFD)
    for failedEx in failedExamples:
      print(failedEx, end=" ", file=mainFD)
    print('</code>', file=mainFD)
    print('  </div>', file=mainFD)
    print('</div>', file=mainFD)

  print('''<div id="UPDATEREFERENCES" class="panel panel-info" style="display:none">
  <div class="panel-heading"><span class="glyphicon glyphicon-pencil">
    </span>&nbsp;<a data-toggle="collapse" href="#collapseUpdateReferences">
 'Update references<span class="caret"> </span></a></div>
  <div class="panel-body panel-collapse collapse" id="collapseUpdateReferences">
    <p>Update the references of the selected examples before next build</p>
    <p>
      <span class="octicon octicon-person"></span>&nbsp;<img id="LOGINUSERIMG" height="20" src="#" alt="avatar">
      <strong id="LOGINUSER">unknwon</strong>
      <button id="LOGINBUTTON" disabled="disabled" type="button"><span class="octicon octicon-sign-in">
        </span>&nbsp;Login <span class="octicon octicon-logo-github"></span></button>
      <button id="LOGOUTBUTTON" disabled="disabled" type="button"><span class="octicon octicon-sign-out"></span>&nbsp;Logout</button>
    </p>
    <p>
      <button id="SAVEBUTTON" disabled="disabled" type="button"><span class="glyphicon glyphicon-ok"></span>&nbsp;Save changes</button>
    </p>
  </div>
</div>
<div id="STATUSPANEL" class="panel panel-info" style="display:none">
  <div class="panel-heading"><span class="glyphicon glyphicon-info-sign">
    </span>&nbsp;<span class="glyphicon glyphicon-exclamation-sign"></span>&nbsp;Status message</div>
  <div class="panel-body">
    <span id="STATUSMSG">Communicating with server, please wait. (reload page if hanging)</span>
  </div>
</div>
<hr/>
<p class="text-right small">
  <a href="http://validator.w3.org/check?uri=referer">
    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>
  </a>
  Generated on %s by runexamples.py
</p>
<span id="FINISHED" style="display:none"> </span>
</body>
</html>'''%(str(timeID)), file=mainFD)

  mainFD.close()
  # replace end time in index.html
  for line in fileinput.FileInput(pj(args.reportOutDir, "index.html"),inplace=1):
    endTime=datetime.datetime.now()
    endTime=datetime.datetime(endTime.year, endTime.month, endTime.day, endTime.hour, endTime.minute, endTime.second)
    line=re.sub('<!--S_ENDTIME-->.*?<!--E_ENDTIME-->', str(endTime), line)
    print(line, end="")

  # write RSS feed
  writeRSSFeed(len(failedExamples), len(retAll))

  # print result summary to console
  if len(failedExamples)>0:
    print('\n'+str(len(failedExamples))+' examples have failed.')
  else:
    print('\nAll examples have passed.')

  return mainRet



#####################################################################################
# from now on only functions follow and at the end main is called
#####################################################################################



def pkgconfig(module, options):
  comm=["pkg-config", module]
  comm.extend(options)
  try:
    output=subprocess.check_output(comm).decode("utf-8")
  except subprocess.CalledProcessError as ex:
    if ex.returncode==0:
      raise
    else:
      print("Error: pkg-config module "+module+" not found. Trying to continue.", file=sys.stderr)
      output="pkg_config_"+module+"_not_found"
  return output.rstrip()



def printFinishedMessage(missingDirectories, result):
  missingDirectories.remove(result[0])
  lenDirs=len(directories)
  curNumber=lenDirs-len(missingDirectories)
  eta=0
  for example in missingDirectories:
    eta+=example[1]
  if not math.isinf(eta):
    etaStr=datetime.timedelta(0, round(eta/min(args.j, multiprocessing.cpu_count())))
  else:
    etaStr="unknown"
  print("Finished example %03d/%03d; %5.1f%%; ETA %s; %s; %s"%(curNumber, lenDirs, curNumber/lenDirs*100,
    etaStr, result[0][0], "passed" if result[2]==0 else "FAILED!!!"))



def sortDirectories(directoriesSet, dirs):
  unsortedDir=[]
  for example in directoriesSet:
    if os.path.isfile(pj(example, "reference", "time.dat")):
      refTimeFD=codecs.open(pj(example, "reference", "time.dat"), "r", encoding="utf-8")
      refTime=float(refTimeFD.read())
      refTimeFD.close()
    else:
      refTime=float("inf") # use very long time if unknown to force it to run early
    unsortedDir.append([example, refTime])
  dirs.extend(sorted(unsortedDir, key=lambda x: x[1], reverse=True))



# handle the --filter option: add/remove to directoriesSet
def addExamplesByFilter(baseDir, directoriesSet):
  # if staring with @ use dirs from file defined by @<filename>: in JSON format
  if baseDir[0]=="@":
    # read file
    fd=open(baseDir[1:], 'r+')
    fcntl.lockf(fd, fcntl.LOCK_EX)
    config=json.load(fd)
    # add examples
    for d in config['checkedExamples']:
      addExamplesByFilter(d, directoriesSet)
    # clear checkedExamples
    config['checkedExamples']=[]
    # write file
    fd.seek(0);
    json.dump(config, fd)
    fd.truncate();
    fcntl.lockf(fd, fcntl.LOCK_UN)
    fd.close()
    return

  if baseDir[0]!="^": # add dir
    addOrDiscard=directoriesSet.add
  else: # remove dir
    baseDir=baseDir[1:] # remove the leading "^"
    addOrDiscard=directoriesSet.discard
  # make baseDir a relative path
  baseDir=os.path.relpath(baseDir)
  for root, dirs, _ in os.walk(baseDir):
    ppxml=os.path.isfile(pj(root, "MBS.mbsimprj.xml"))
    flatxml=os.path.isfile(pj(root, "MBS.mbsimprj.flat.xml"))
    xml=ppxml or flatxml
    src=os.path.isfile(pj(root, "Makefile"))
    fmi=(os.path.isfile(pj(root, "FMI.mbsimprj.xml")) or os.path.isfile(pj(root, "Makefile_FMI")))
    # skip none examples directires
    if(not ppxml and not flatxml and not src and not fmi):
      continue
    dirs=[]
    d={'ppxml': ppxml, 'flatxml': flatxml, 'xml': xml, 'src': src, 'fmi': fmi}
    for m in mbsimModules:
      d[m]=False
    # check for MBSim modules in src examples
    if src:
      filecont=codecs.open(pj(root, "Makefile"), "r", encoding="utf-8").read()
      for m in mbsimModules:
        if re.search("\\b"+m+"\\b", filecont): d[m]=True
    # check for MBSim modules in xml and flatxml examples
    else:
      for filedir, _, filenames in os.walk(root):
        for filename in fnmatch.filter(filenames, "*.xml"):
          if filename[0:4]==".pp.": continue # skip generated .pp.* files
          filecont=codecs.open(pj(filedir, filename), "r", encoding="utf-8").read()
          for m in mbsimModules:
            if re.search('=\\s*"http://[^"]*'+m+'"', filecont, re.I): d[m]=True
    # evaluate filter
    try:
      filterResult=eval(args.filter, d)
    except:
      print("Unable to evaluate the filter:\n"+args.filter)
      exit(1)
    if filterResult:
      addOrDiscard(os.path.normpath(root))



# run the given example
def runExample(resultQueue, example):
  savedDir=os.getcwd()
  try:
    resultStr=""
    os.chdir(example[0])

    runExampleRet=0 # run ok
    # execute the example[0]
    if not os.path.isdir(pj(args.reportOutDir, example[0])): os.makedirs(pj(args.reportOutDir, example[0]))
    executeFN=pj(example[0], "execute.txt")
    executeRet=0
    if not args.disableRun:
      # clean output of previous run
      if os.path.isfile("time.dat"): os.remove("time.dat")
      list(map(os.remove, glob.glob("*.h5")))

      executeFD=MultiFile(codecs.open(pj(args.reportOutDir, executeFN), "w", encoding="utf-8"), args.printToConsole)
      dt=0
      if os.path.isfile("Makefile"):
        executeRet, dt, outfiles=executeSrcExample(executeFD, example)
      elif os.path.isfile("MBS.mbsimprj.xml"):
        executeRet, dt, outfiles=executeXMLExample(executeFD, example)
      elif os.path.isfile("MBS.mbsimprj.flat.xml"):
        executeRet, dt, outfiles=executeFlatXMLExample(executeFD, example)
      elif os.path.isfile("FMI.mbsimprj.xml"):
        executeRet, dt, outfiles=executeFMIXMLExample(executeFD, example)
      elif os.path.isfile("Makefile_FMI"):
        executeRet, dt, outfiles=executeFMISrcExample(executeFD, example)
      else:
        print("Unknown example type in directory "+example[0]+" found.", file=executeFD)
        executeRet=1
        dt=0
        outfiles=[]
      executeFD.close()
    if executeRet==None or executeRet!=0: runExampleRet=1
    # get reference time
    refTime=example[1]
    # print result to resultStr
    if not example[0] in willFail:
      resultStr+='<tr>'
    else:
      resultStr+='<tr class="text-muted">'
    resultStr+='<td>'+example[0].replace('/', u'/\u200B')+'</td>'
    if not args.disableRun:
      if executeRet==None or executeRet!=0:
        resultStr+='<td class="danger"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;<a href="'+myurllib.pathname2url(executeFN)+'">'
      else:
        resultStr+='<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;<a href="'+myurllib.pathname2url(executeFN)+'">'
      if executeRet==None:
        resultStr+='timed out'
      elif executeRet!=0:
        resultStr+='failed'
      else:
        resultStr+='passed'
      resultStr+='</a>'
      # add all additional output files
      for outfile in outfiles:
        resultStr+='; <a href="'+myurllib.pathname2url(pj(example[0], outfile))+'">'+outfile+'</a>'
      resultStr+='</td>'
    if not args.disableRun:
      # if not reference time or time is nearly equal refTime => display time in black color
      if math.isinf(refTime) or abs(dt-refTime)<0.1*refTime:
        resultStr+='<td>%.3f</td>'%dt
      # dt differs more then 10% from refTime => display in yellow color
      else:
        resultStr+='<td class="%s">%.3f</td>'%("success" if dt<refTime else "warning", dt)
      if not math.isinf(refTime):
        resultStr+='<td>%.3f</td>'%refTime
      else:
        resultStr+='<td class="warning">no reference</td>'

    compareRet=-1
    compareFN=pj(example[0], "compare.html")
    if not args.disableCompare and canCompare:
      if os.path.isdir("reference"):
        # compare the result with the reference
        compareRet, nrFailed, nrAll=compareExample(example[0], compareFN)
        if compareRet!=0: runExampleRet=1
      else:
        compareRet=-2

    # write time to time.dat for possible later copying it to the reference
    if not args.disableRun:
      refTimeFD=codecs.open("time.dat", "w", encoding="utf-8")
      print('%.3f'%dt, file=refTimeFD)
      refTimeFD.close()
    # print result to resultStr
    if not args.disableCompare:
      if compareRet==-1:
        resultStr+='<td class="warning"><div class="pull-left"><span class="glyphicon glyphicon-warning-sign alert-warning"></span>&nbsp;not run</div>'+\
                   '<div class="pull-right">[<input type="checkbox" disabled="disabled"/>]</div></td>'
      elif compareRet==-2:
        resultStr+='<td class="warning"><div class="pull-left"><span class="glyphicon glyphicon-warning-sign alert-warning"></span>&nbsp;no reference</div>'+\
                   '<div class="pull-right">[<input class="_EXAMPLE'+\
                   '" type="checkbox" name="'+example[0]+'" disabled="disabled"/>]</div></td>'
        nrAll=0
        nrFailed=0
      else:
        if nrFailed==0:
          resultStr+='<td class="success"><div class="pull-left"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;<a href="'+myurllib.pathname2url(compareFN)+\
                     '">passed <span class="badge">'+str(nrAll)+'</span></a></div>'+\
                     '<div class="pull-right">[<input type="checkbox" disabled="disabled"/>]</div></td>'
        else:
          resultStr+='<td class="danger"><div class="pull-left"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;<a href="'+myurllib.pathname2url(compareFN)+\
                     '">failed <span class="badge">'+str(nrFailed)+'</span> of <span class="badge">'+str(nrAll)+\
                     '</span></a></div><div class="pull-right">[<input class="_EXAMPLE'+\
                     '" type="checkbox" name="'+example[0]+'" disabled="disabled"/>]</div></td>'

    # check for deprecated features
    if not args.disableRun:
      nrDeprecated=0
      for line in fileinput.FileInput(pj(args.reportOutDir, executeFN)):
        match=re.search("([0-9]+) deprecated features were called:", line)
        if match!=None:
          nrDeprecated=match.expand("\\1")
          break
      if nrDeprecated==0:
        resultStr+='<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;none</td>'
      else:
        resultStr+='<td class="warning"><span class="glyphicon glyphicon-warning-sign alert-warning"></span>&nbsp;<a href="'+myurllib.pathname2url(executeFN)+'">'+str(nrDeprecated)+' found</a></td>'

    # validate XML
    if not args.disableValidate:
      htmlOutputFN=pj(example[0], "validateXML.html")
      htmlOutputFD=codecs.open(pj(args.reportOutDir, htmlOutputFN), "w", encoding="utf-8")
      # write header
      print('<!DOCTYPE html>', file=htmlOutputFD)
      print('<html lang="en">', file=htmlOutputFD)
      print('<head>', file=htmlOutputFD)
      print('  <META http-equiv="Content-Type" content="text/html; charset=UTF-8">', file=htmlOutputFD)
      print('  <meta name="viewport" content="width=device-width, initial-scale=1.0" />', file=htmlOutputFD)
      print('  <title>Validate XML Files</title>', file=htmlOutputFD)
      print('  <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>', file=htmlOutputFD)
      print('  <link rel="stylesheet" href="http://cdn.datatables.net/1.10.2/css/jquery.dataTables.css"/>', file=htmlOutputFD)
      print('</head>', file=htmlOutputFD)
      print('<body style="margin:1em">', file=htmlOutputFD)
      print('<script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>', file=htmlOutputFD)
      print('<script type="text/javascript" src="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/js/bootstrap.min.js"> </script>', file=htmlOutputFD)
      print('<script type="text/javascript" src="http://cdn.datatables.net/1.10.2/js/jquery.dataTables.min.js"> </script>', file=htmlOutputFD)
      print('<script type="text/javascript">', file=htmlOutputFD)
      print('  $(document).ready(function() {', file=htmlOutputFD)
      print("    $('#SortThisTable').dataTable({'lengthMenu': [ [10, 25, 50, 100, -1], [10, 25, 50, 100, 'All'] ], 'pageLength': 25, 'aaSorting': [], stateSave: true});", file=htmlOutputFD)
      print('  } );', file=htmlOutputFD)
      print('</script>', file=htmlOutputFD)
      print('<h1>Validate XML Files</h1>', file=htmlOutputFD)
      print('<dl class="dl-horizontal">', file=htmlOutputFD)
      print('<dt>Example:</dt><dd>'+example[0]+'</dd>', file=htmlOutputFD)
      print('<dt>Time ID:</dt><dd>'+str(timeID)+'</dd>', file=htmlOutputFD)
      currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
      parDirs="/".join(list(map(lambda x: "..", range(0, example[0].count(os.sep)+1))))
      navA=""
      navB=""
      if args.currentID!=0:
        navA="/../.."
        navB="/runexamples_report/result_current"
      print('<dt>Navigate:</dt><dd><a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-backward"> </span> previous</a>'%
        (parDirs, navA, currentID-1, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-forward"> </span> next</a>'%
        (parDirs, navA, currentID+1, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_current%s/%s"><span class="glyphicon glyphicon-fast-forward"> </span> newest</a>'%
        (parDirs, navA, navB, myurllib.pathname2url(htmlOutputFN)), file=htmlOutputFD)
      print('                 <a class="btn btn-info btn-xs" href="%s%s%s/index.html"><span class="glyphicon glyphicon-eject"> </span> parent</a></dd>'%
        (parDirs, navA, navB), file=htmlOutputFD)
      print('</dl>', file=htmlOutputFD)
      print('<hr/><table id="SortThisTable" class="table table-striped table-hover table-bordered table-condensed">', file=htmlOutputFD)
      print('<thead><tr><th><span class="glyphicon glyphicon-folder-open"></span>&nbsp;XML File</th>'+
            '<th><span class="glyphicon glyphicon-search"></span>&nbsp;Result</th></tr></thead><tbody>', file=htmlOutputFD)

      failed, total=validateXML(example, False, htmlOutputFD)
      if failed==0:
        resultStr+='<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;<a href="'+myurllib.pathname2url(htmlOutputFN)+'">valid <span class="badge">'+\
                   str(total)+'</span></a></td>'
      else:
        resultStr+='<td class="danger"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;<a href="'+myurllib.pathname2url(htmlOutputFN)+'">'+\
                   'failed <span class="badge">'+str(failed)+'</span> of <span class="badge">'+str(total)+'</span></a></td>'
        runExampleRet=1
      # write footer
      print('</tbody></table>', file=htmlOutputFD)
      print('<hr/>', file=htmlOutputFD)
      print('<p class="text-right small">', file=htmlOutputFD)
      print('  <a href="http://validator.w3.org/check?uri=referer">', file=htmlOutputFD)
      print('    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>', file=htmlOutputFD)
      print('  </a>', file=htmlOutputFD)
      print('  Generated on %s by runexamples.py'%(str(timeID)), file=htmlOutputFD)
      print('</p>', file=htmlOutputFD)
      print('</body>', file=htmlOutputFD)
      print('</html>', file=htmlOutputFD)

      htmlOutputFD.close()

    resultStr+='</tr>'

  except:
    fatalScriptErrorFN=pj(example[0], "fatalScriptError.txt")
    fatalScriptErrorFD=MultiFile(codecs.open(pj(args.reportOutDir, fatalScriptErrorFN), "w", encoding="utf-8"), args.printToConsole)
    print("Fatal Script Errors should not happen. So this is a bug in runexamples.py which should be fixed.", file=fatalScriptErrorFD)
    print("", file=fatalScriptErrorFD)
    print(traceback.format_exc(), file=fatalScriptErrorFD)
    fatalScriptErrorFD.close()
    resultStr='<tr><td>'+example[0]+'</td><td class="danger"><a href="'+myurllib.pathname2url(fatalScriptErrorFN)+'">fatal script error</a></td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td></tr>'
    runExampleRet=1
  finally:
    os.chdir(savedDir)
    resultQueue.put([example, resultStr, runExampleRet])
    return runExampleRet



# prefix the simultion with this parameter.
# this is normaly just args.prefixSimulation but may be extended by keywords of args.prefixSimulationKeyword.
def prefixSimulation(example, id):
  # handle VALGRIND
  if args.prefixSimulationKeyword=='VALGRIND':
    return args.prefixSimulation+['--xml=yes', '--xml-file=valgrind.%%p.%s.xml'%(id)]
  return args.prefixSimulation

# get additional output files of simulations.
# these are all dependent on the keyword of args.prefixSimulationKeyword.
# additional output files must be placed in the args.reportOutDir and here only the basename must be returned.
def getOutFilesAndAdaptRet(example, ret):
  # handle VALGRIND
  if args.prefixSimulationKeyword=='VALGRIND':
    # get out files
    # and adapt the return value if errors in valgrind outputs are detected
    xmlFiles=glob.glob("valgrind.*.xml")
    outFiles=[]
    for xmlFile in xmlFiles:
      # check for errors
      content=codecs.open(xmlFile).read().decode('utf-8')
      if "</valgrindoutput>" not in content: # incomplete valgrind output -> a skipped trace children
        os.remove(xmlFile)
        continue
      if "<error>" in content and ret[0]!=None:
        ret[0]=1
      # transform xml file to html file (in reportOutDir)
      htmlFile=xmlFile[:-4]+".html"
      outFiles.append(htmlFile)
      global scriptDir
      subprocess.call(['Xalan', '-o', pj(args.reportOutDir, example[0], htmlFile), xmlFile,
                       pj(scriptDir, 'valgrindXMLToHTML.xsl')])
      os.remove(xmlFile)
    return outFiles
  return []



# execute the source code example in the current directory (write everything to fd executeFD)
def executeSrcExample(executeFD, example):
  print("Running commands:", file=executeFD)
  print("make clean && make && "+pj(os.curdir, "main"), file=executeFD)
  print("", file=executeFD)
  executeFD.flush()
  if not args.disableMakeClean:
    if subprocessCall(["make", "clean"], executeFD)!=0: return 1, 0, []
  if subprocessCall(["make"], executeFD)!=0: return 1, 0, []
  # append $prefix/lib to LD_LIBRARY_PATH/PATH to find lib by main of the example
  if os.name=="posix":
    NAME="LD_LIBRARY_PATH"
    SUBDIR="lib"
  elif os.name=="nt":
    NAME="PATH"
    SUBDIR="bin"
  mainEnv=os.environ
  libDir=pj(mbsimBinDir, os.pardir, SUBDIR)
  if NAME in mainEnv:
    mainEnv[NAME]=mainEnv[NAME]+os.pathsep+libDir
  else:
    mainEnv[NAME]=libDir
  # run main
  t0=datetime.datetime.now()
  ret=[subprocessCall(prefixSimulation(example, 'src')+[pj(os.curdir, "main"+args.exeExt)], executeFD,
                      env=mainEnv, maxExecutionTime=args.maxExecutionTime)]
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  outFiles=getOutFilesAndAdaptRet(example, ret)
  return ret[0], dt, outFiles



# execute the XML example in the current directory (write everything to fd executeFD)
def executeXMLExample(executeFD, example):
  # we handle MBS.mbsimprj.xml and FMI.mbsimprj.xml files here
  prjFile=glob.glob("*.mbsimprj.xml")[0]

  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimxml")]+[prjFile]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  ret=[subprocessCall(prefixSimulation(example, 'xml')+[pj(mbsimBinDir, "mbsimxml"+args.exeExt)]+
                      [prjFile], executeFD, maxExecutionTime=args.maxExecutionTime)]
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  outFiles=getOutFilesAndAdaptRet(example, ret)
  return ret[0], dt, outFiles



# execute the flat XML example in the current directory (write everything to fd executeFD)
def executeFlatXMLExample(executeFD, example):
  print("Running command:", file=executeFD)
  list(map(lambda x: print(x, end=" ", file=executeFD), [pj(mbsimBinDir, "mbsimflatxml"), "MBS.mbsimprj.flat.xml"]))
  print("\n", file=executeFD)
  executeFD.flush()
  t0=datetime.datetime.now()
  ret=[subprocessCall(prefixSimulation(example, 'fxml')+[pj(mbsimBinDir, "mbsimflatxml"+args.exeExt), "MBS.mbsimprj.flat.xml"],
                      executeFD, maxExecutionTime=args.maxExecutionTime)]
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  outFiles=getOutFilesAndAdaptRet(example, ret)
  return ret[0], dt, outFiles



# helper function for executeFMIXMLExample and executeFMISrcExample
def executeFMIExample(executeFD, example, fmiInputFile):
  # run mbsimCreateFMU to export the model as a FMU
  # use option --nocompress, just to speed up mbsimCreateFMU
  print("\n\n\n", file=executeFD)
  print("Running command:", file=executeFD)
  comm=[pj(mbsimBinDir, "mbsimCreateFMU"+args.exeExt), '--nocompress', fmiInputFile]
  list(map(lambda x: print(x, end=" ", file=executeFD), comm))
  print("\n", file=executeFD)
  executeFD.flush()
  ret1=[subprocessCall(prefixSimulation(example, 'fmucre')+comm, executeFD, maxExecutionTime=args.maxExecutionTime/5)]
  outFiles1=getOutFilesAndAdaptRet(example, ret1)

  # get fmuChecker executable
  fmuCheck=glob.glob(pj(mbsimBinDir, "fmuCheck.*"))
  if len(fmuCheck)!=1:
    raise RuntimeError("None or more than one fmuCheck.* executlabe found.")
  fmuCheck=fmuCheck[0]
  # run fmuChecker
  print("\n\n\n", file=executeFD)
  print("Running command:", file=executeFD)
  # adapt end time if MBSIM_SET_MINIMAL_TEND is set
  endTime=[]
  if 'MBSIM_SET_MINIMAL_TEND' in os.environ:
    endTime=['-s', '0.01']
  comm=[pj(mbsimBinDir, fmuCheck)]+endTime+["-l", "5", "mbsim.fmu"]
  list(map(lambda x: print(x, end=" ", file=executeFD), comm))
  print("\n", file=executeFD)
  t0=datetime.datetime.now()
  ret2=[subprocessCall(prefixSimulation(example, 'fmuchk')+comm, executeFD, maxExecutionTime=args.maxExecutionTime)]
  t1=datetime.datetime.now()
  dt=(t1-t0).total_seconds()
  outFiles2=getOutFilesAndAdaptRet(example, ret2)

  # return
  if ret1[0]==None or ret2[0]==None:
    ret=None
  else:
    ret=abs(ret1[0])+abs(ret2[0])
  outFiles=[]
  outFiles.extend(outFiles1)
  outFiles.extend(outFiles2)
  return ret, dt, outFiles

# execute the FMI XML export example in the current directory (write everything to fd executeFD)
def executeFMIXMLExample(executeFD, example):
  # first simple run the example as a preprocessing xml example
  ret1, dt, outFiles1=executeXMLExample(executeFD, example)
  # create and run FMU
  ret2, dt, outFiles2=executeFMIExample(executeFD, example, "FMI.mbsimprj.xml")
  # return
  ret=abs(ret1)+abs(ret2)
  outFiles=[]
  outFiles.extend(outFiles1)
  outFiles.extend(outFiles2)
  return ret, dt, outFiles

# execute the FMI source export example in the current directory (write everything to fd executeFD)
def executeFMISrcExample(executeFD, example):
  # compile examples
  print("Running commands:", file=executeFD)
  print("make -f Makefile_FMI clean && make -f Makefile_FMI", file=executeFD)
  print("", file=executeFD)
  executeFD.flush()
  if not args.disableMakeClean:
    if subprocessCall(["make", "-f", "Makefile_FMI", "clean"], executeFD)!=0: return 1, 0, []
  if subprocessCall(["make", "-f", "Makefile_FMI"], executeFD)!=0: return 1, 0, []
  # create and run FMU
  if args.exeExt==".exe":
    dllExt=".dll"
  else:
    dllExt=".so"
  return executeFMIExample(executeFD, example, "mbsimfmi_model"+dllExt)



def createDiffPlot(diffHTMLFileName, example, filename, datasetName, column, label, dataArrayRef, dataArrayCur, gnuplotProcess):
  import numpy

  diffDir=os.path.dirname(diffHTMLFileName)
  if not os.path.isdir(diffDir): os.makedirs(diffDir)

  # create html page
  diffHTMLPlotFD=codecs.open(diffHTMLFileName, "w", encoding="utf-8")
  print('<!DOCTYPE html>', file=diffHTMLPlotFD)
  print('<html lang="en">', file=diffHTMLPlotFD)
  print('<head>', file=diffHTMLPlotFD)
  print('  <META http-equiv="Content-Type" content="text/html; charset=UTF-8">', file=diffHTMLPlotFD)
  print('  <meta name="viewport" content="width=device-width, initial-scale=1.0" />', file=diffHTMLPlotFD)
  print('  <title>Difference Plot</title>', file=diffHTMLPlotFD)
  print('  <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>', file=diffHTMLPlotFD)
  print('</head>', file=diffHTMLPlotFD)
  print('<body style="margin:1em">', file=diffHTMLPlotFD)
  print('<h1>Difference Plot</h1>', file=diffHTMLPlotFD)
  print('<dl class="dl-horizontal">', file=diffHTMLPlotFD)
  print('<dt>Example:</dt><dd>'+example+'</dd>', file=diffHTMLPlotFD)
  print('<dt>File:</dt><dd>'+filename+'</dd>', file=diffHTMLPlotFD)
  print('<dt>Dataset:</dt><dd>'+datasetName+'</dd>', file=diffHTMLPlotFD)
  print('<dt>Label:</dt><dd>'+label+' (column %d)</dd>'%(column), file=diffHTMLPlotFD)
  print('<dt>Time ID:</dt><dd>'+str(timeID)+'</dd>', file=diffHTMLPlotFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  parDirs="/".join(list(map(lambda x: "..", range(0, pj(example, filename, datasetName, str(column)).count(os.sep)+1))))
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('<dt>Navigate:</dt><dd><a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-backward"> </span> previous</a>'%
    (parDirs, navA, currentID-1, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-forward"> </span> next</a>'%
    (parDirs, navA, currentID+1, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_current%s/%s"><span class="glyphicon glyphicon-fast-forward"> </span> newest</a>'%
    (parDirs, navA, navB, example+"/"+filename+"/"+datasetName+"/"+str(column)+"/diffplot.html"), file=diffHTMLPlotFD)
  print('                 <a class="btn btn-info btn-xs" href="%s/%s%s%s/compare.html"><span class="glyphicon glyphicon-eject"> </span> parent</a></dd>'%
    (parDirs, myurllib.pathname2url(example), navA, navB), file=diffHTMLPlotFD)
  print('</dl>', file=diffHTMLPlotFD)
  print('<p><span class="glyphicon glyphicon-info-sign"> </span> A result differs if <b>at least at one time point</b> the absolute tolerance <b>and</b> the relative tolerance is larger then the requested.</p>', file=diffHTMLPlotFD)
  print('<p><object data="plot.svg" type="image/svg+xml"> </object></p>', file=diffHTMLPlotFD)
  print('<hr/>', file=diffHTMLPlotFD)
  print('<p class="text-right small">', file=diffHTMLPlotFD)
  print('  <a href="http://validator.w3.org/check?uri=referer">', file=diffHTMLPlotFD)
  print('    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>', file=diffHTMLPlotFD)
  print('  </a>', file=diffHTMLPlotFD)
  print('  Generated on %s by runexamples.py'%(str(timeID)), file=diffHTMLPlotFD)
  print('</p>', file=diffHTMLPlotFD)
  print('</body>', file=diffHTMLPlotFD)
  print('</html>', file=diffHTMLPlotFD)
  diffHTMLPlotFD.close()

  if gnuplotProcess==None:
    return

  # fix if all values are nan to prevent a gnuplot warning
  if numpy.all(numpy.isnan(dataArrayRef[:,0])) or numpy.all(numpy.isnan(dataArrayRef[:,1])) or \
     numpy.all(numpy.isnan(dataArrayCur[:,0])) or numpy.all(numpy.isnan(dataArrayCur[:,1])):
    add=numpy.array([
      [float("nan"), float("nan")],
      [0, 0],
    ])
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)
    dataArrayRef=numpy.concatenate((dataArrayRef, add), axis=0)

  # get minimum of all x and y values (and add some space as border)
  (xmin,ymin)=numpy.minimum(numpy.nanmin(dataArrayRef,0), numpy.nanmin(dataArrayCur,0))
  (xmax,ymax)=numpy.maximum(numpy.nanmax(dataArrayRef,0), numpy.nanmax(dataArrayCur,0))
  dx=(xmax-xmin)/50
  dy=(ymax-ymin)/50
  if dx==0: dx=1
  if dy==0: dy=1
  xmin=xmin-dx
  ymin=ymin-dy
  xmax=xmax+dx
  ymax=ymax+dy

  # create datafile
  dataFileName=pj(diffDir, "data.dat")
  nradd=dataArrayRef.shape[0]-dataArrayCur.shape[0]
  add=numpy.empty([abs(nradd), 2])
  add[:]=float("NaN")
  if nradd<0:
    dataArrayRef=numpy.concatenate((dataArrayRef, add), axis=0)
  if nradd>0:
    dataArrayCur=numpy.concatenate((dataArrayCur, add), axis=0)
  dataArrayRefCur=numpy.concatenate((dataArrayRef, dataArrayCur), axis=1)
  dataArrayRefCur.tofile(dataFileName)

  # create gnuplot file
  SVGFileName=pj(diffDir, "plot.svg")
  gnuplotProcess.stdin.write(("set output '"+SVGFileName+"'\n").encode("utf-8"))
  gnuplotProcess.stdin.write(("set multiplot layout 3, 1\n").encode("utf-8"))
  gnuplotProcess.stdin.write(("set title 'Compare'\n").encode("utf-8"))
  gnuplotProcess.stdin.write(("set ylabel 'Value'\n").encode("utf-8"))
  gnuplotProcess.stdin.write(("plot [%g:%g] [%g:%g] \\\n"%(xmin,xmax, ymin,ymax)).encode("utf-8"))
  gnuplotProcess.stdin.write(("  '"+dataFileName+"' u ($1):($2) binary format='%double%double%double%double' title 'ref' w l lw 2, \\\n").encode("utf-8"))
  gnuplotProcess.stdin.write(("  '"+dataFileName+"' u ($3):($4) binary format='%double%double%double%double' title 'cur' w l\n").encode("utf-8"))
  if dataArrayRef.shape==dataArrayCur.shape:
    gnuplotProcess.stdin.write(("set title 'Absolute Tolerance'\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("set ylabel 'cur-ref'\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("plot [%g:%g] [%g:%g] \\\n"%(xmin,xmax, -3*args.atol, 3*args.atol)).encode("utf-8"))
    gnuplotProcess.stdin.write(("  '"+dataFileName+"' u ($1):($4-$2) binary format='%double%double%double%double' title 'cur-ref' w l, \\\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("  %g title 'atol' lt 2 lw 1, \\\n"%(args.atol)).encode("utf-8"))
    gnuplotProcess.stdin.write(("  %g notitle lt 2 lw 1\n"%(-args.atol)).encode("utf-8"))
    gnuplotProcess.stdin.write(("set title 'Relative Tolerance'\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("set ylabel '(cur-ref)/ref'\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("plot [%g:%g] [%g:%g] \\\n"%(xmin,xmax, -3*args.rtol, 3*args.rtol)).encode("utf-8"))
    # prevent division by zero: use 1e-30 instead
    gnuplotProcess.stdin.write(("  '"+dataFileName+"' u ($1):(($4-$2)/($2==0?1e-30:$2)) binary format='%double%double%double%double' title '(cur-ref)/ref' w l, \\\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("  %g title 'rtol' lt 2 lw 1, \\\n"%(args.rtol)).encode("utf-8"))
    gnuplotProcess.stdin.write(("  %g notitle lt 2 lw 1\n"%(-args.rtol)).encode("utf-8"))
  gnuplotProcess.stdin.write(("unset multiplot\n").encode("utf-8"))
  # dataFileName it not removed since gnuplot is running asynchronously

# numpy.isclose(...) is only defined in newer numpy versions.
# This implementation is taken varbatim from numpy 1.9
def numpy_isclose(a, b, rtol=1.e-5, atol=1.e-8, equal_nan=False):
  import numpy
  def within_tol(x, y, atol, rtol):
    with numpy.errstate(invalid='ignore'):
      result = numpy.less_equal(abs(x-y), atol + rtol * abs(y))
    if numpy.isscalar(a) and numpy.isscalar(b):
      result = bool(result)
    return result
  x = numpy.array(a, copy=False, subok=True, ndmin=1)
  y = numpy.array(b, copy=False, subok=True, ndmin=1)
  xfin = numpy.isfinite(x)
  yfin = numpy.isfinite(y)
  if all(xfin) and all(yfin):
    return within_tol(x, y, atol, rtol)
  else:
    finite = xfin & yfin
    cond = numpy.zeros_like(finite, subok=True)
    x = x * numpy.ones_like(cond)
    y = y * numpy.ones_like(cond)
    cond[finite] = within_tol(x[finite], y[finite], atol, rtol)
    cond[~finite] = (x[~finite] == y[~finite])
    if equal_nan:
      both_nan = numpy.isnan(x) & numpy.isnan(y)
      cond[both_nan] = both_nan[both_nan]
    return cond
# return column col from arr as a column Vector if asColumnVector == True or as a row vector
# arr may be of shape vector or a matrix
def getColumn(arr, col, asColumnVector=True):
  if len(arr.shape)==2:
    if asColumnVector:
      return arr[:,col]
    else:
      return arr[:,col:col+1]
  elif len(arr.shape)==1 and col==0:
    if asColumnVector:
      return arr[:]
    else:
      return arr[:][:,None]
  else:
    raise IndexError("Only HDF5 datasets of shape vector and matrix can be handled.")
def compareDatasetVisitor(h5CurFile, data, example, nrAll, nrFailed, refMemberNames, gnuplotProcess, datasetName, refObj):
  import numpy
  import h5py

  if isinstance(refObj, h5py.Dataset):
    # add to refMemberNames
    refMemberNames.add(datasetName)
    # the corresponding curObj to refObj
    try:
      curObj=h5CurFile[datasetName]
    except KeyError:
      data.append([
        (h5CurFile.filename,""),
        (datasetName,""),
        ('in ref. but not in cur.',"d"),
        ('failed',"d")
      ])
      nrAll[0]+=1
      nrFailed[0]+=1
      return
    # get shape
    refObjCols=refObj.shape[1] if len(refObj.shape)==2 else 1
    curObjCols=curObj.shape[1] if len(curObj.shape)==2 else 1
    # get labels from reference
    try:
      refLabels=list(map(lambda x: (x.decode("utf-8"), 's'), refObj.attrs["Column Label"]))
      # append missing dummy labels
      for x in range(len(refLabels), refObjCols):
        refLabels.append(('&lt;no label in ref. for col. '+str(x+1)+'&gt;', 'w'))
    except KeyError:
      refLabels=list(map(
        lambda x: ('&lt;no label for col. '+str(x+1)+'&gt;', 'w'),
        range(refObjCols)))
    # get labels from current
    try:
      curLabels=list(map(lambda x: (x.decode("utf-8"), 'success'), curObj.attrs["Column Label"]))
      # append missing dummy labels
      for x in range(len(curLabels), curObjCols):
        curLabels.append(('&lt;no label in cur. for col. '+str(x+1)+'&gt;', 'warning'))
    except KeyError:
      curLabels=list(map(
        lambda x: ('&lt;no label for col. '+str(x+1)+'&gt;', 'warning'),
        range(refObjCols)))
    # loop over all columns
    for column in range(refObjCols):
      printLabel=refLabels[column]
      diffFilename=pj(h5CurFile.filename, datasetName, str(column), "diffplot.html")
      nrAll[0]+=1
      # if if curObj[:,column] does not exitst
      if column>=curObjCols:
        printLabel=('&lt;label '+printLabel[0]+' not in cur.&gt;', 'd')
        nrFailed[0]+=1
      cell=[]
      cell.append((h5CurFile.filename,""))
      cell.append((datasetName,""))
      if column<curObjCols and refLabels[column][0]==curLabels[column][0]:
        cell.append((printLabel[0],printLabel[1]))
      else:
        cell.append(('&lt;label for col. '+str(column+1)+' differ&gt;',"d"))
        nrFailed[0]+=1
      if column<curObjCols and curObj.shape[0]>0 and curObj.shape[0]>0: # only if curObj and refObj contains data (rows)
        # check for difference
        refObjCol=getColumn(refObj,column)
        curObjCol=getColumn(curObj,column)
        if refObjCol.shape[0]==curObjCol.shape[0] and not numpy.all(numpy_isclose(refObjCol, curObjCol, rtol=args.rtol,
                         atol=args.atol, equal_nan=True)):
          cell.append(('<a href="'+myurllib.pathname2url(diffFilename)+'">failed</a>',"d"))
          nrFailed[0]+=1
          dataArrayRef=numpy.concatenate((getColumn(refObj, 0, False), getColumn(refObj, column, False)), axis=1)
          dataArrayCur=numpy.concatenate((getColumn(curObj, 0, False), getColumn(curObj, column, False)), axis=1)
          createDiffPlot(pj(args.reportOutDir, example, diffFilename), example, h5CurFile.filename, datasetName,
                         column, refLabels[column][0], dataArrayRef, dataArrayCur, gnuplotProcess)
        # everything OK
        else:
          cell.append(('passed',"s"))
      else: # not row in curObj or refObj
        cell.append(('no data row in cur. or ref.',"w"))
      data.append(cell)
    # check for labels/columns in current but not in reference
    for label in curLabels[len(refLabels):]:
      data.append([
        (h5CurFile.filename,""),
        (datasetName,""),
        ('label '+label+' not in ref.',"d"),
        ('failed',"d")
      ])
      nrAll[0]+=1
      nrFailed[0]+=1

def appendDatasetName(curMemberNames, datasetName, curObj):
  import h5py
  if isinstance(curObj, h5py.Dataset):
    # add to curMemberNames
    curMemberNames.add(datasetName)

# compare the example with the reference solution
def compareExample(example, compareFN):
  import h5py

  compareFD=codecs.open(pj(args.reportOutDir, compareFN), "w", encoding="utf-8")

  # print html header
  print('<!DOCTYPE html>', file=compareFD)
  print('<html lang="en">', file=compareFD)
  print('<head>', file=compareFD)
  print('  <META http-equiv="Content-Type" content="text/html; charset=UTF-8">', file=compareFD)
  print('  <meta name="viewport" content="width=device-width, initial-scale=1.0" />', file=compareFD)
  print('  <title>Compare Results</title>', file=compareFD)
  print('  <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>', file=compareFD)
  print('  <link rel="stylesheet" href="http://cdn.datatables.net/1.10.2/css/jquery.dataTables.css"/>', file=compareFD)
  print('</head>', file=compareFD)
  print('<body style="margin:1em">', file=compareFD)
  print('<script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>', file=compareFD)
  print('<script type="text/javascript" src="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/js/bootstrap.min.js"> </script>', file=compareFD)
  print('<script type="text/javascript" src="http://cdn.datatables.net/1.10.2/js/jquery.dataTables.min.js"> </script>', file=compareFD)
  print('''<script type="text/javascript">
    $(document).ready(function() {
      $('#SortThisTable').dataTable({
        'lengthMenu': [ [10, 25, 50, 100, -1], [10, 25, 50, 100, 'All'] ],
        'pageLength': 25,
        'aaSorting': [],
        'stateSave': true,
        // we use javascript source for this table due to performance reasons
        'data': SortThisTable_data,
        'columns': [
          { data: 'd0' },
          { data: 'd1' },
          { data: 'd2' },
          { data: 'd3' }
        ],
        "rowCallback": function(row, data) {
          var alltd=$(row).children("td");
          for(var c=2; c<4; c++) {
            var td=alltd.eq(c);
            var flag=data["c"+c.toString()];
            if(flag=="w") {
              td.addClass("warning");
              td.children("span.glyphicon").remove();
              td.prepend('<span class="glyphicon glyphicon-warning-sign alert-warning"></span> ');
            }
            if(flag=="d") {
              td.addClass("danger");
              td.children("span.glyphicon").remove();
              td.prepend('<span class="glyphicon glyphicon-exclamation-sign alert-danger"></span> ');
            }
            if(flag=="s") {
              td.addClass("success");
              td.children("span.glyphicon").remove();
              td.prepend('<span class="glyphicon glyphicon-ok-sign alert-success"></span> ');
            }
          }
        }
      });
      $('#SortThisTable').DataTable().columns.adjust().draw();
    });
    </script>''', file=compareFD)
  print('<h1>Compare Results</h1>', file=compareFD)
  print('<dl class="dl-horizontal">', file=compareFD)
  print('<dt>Example:</dt><dd>'+example+'</dd>', file=compareFD)
  print('<dt>Time ID:</dt><dd>'+str(timeID)+'</dd>', file=compareFD)
  currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
  parDirs="/".join(list(map(lambda x: "..", range(0, example.count(os.sep)+1))))
  navA=""
  navB=""
  if args.currentID!=0:
    navA="/../.."
    navB="/runexamples_report/result_current"
  print('<dt>Navigate:</dt><dd><a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-backward"> </span> previous</a>'%
    (parDirs, navA, currentID-1, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_%010d%s/%s"><span class="glyphicon glyphicon-step-forward"> </span> next</a>'%
    (parDirs, navA, currentID+1, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a class="btn btn-info btn-xs" href="%s/..%s/result_current%s/%s"><span class="glyphicon glyphicon-fast-forward"> </span> newest</a>'%
    (parDirs, navA, navB, myurllib.pathname2url(pj(example, "compare.html"))), file=compareFD)
  print('                 <a class="btn btn-info btn-xs" href="%s%s%s/index.html"><span class="glyphicon glyphicon-eject"> </span> parent</a></dd>'%
    (parDirs, navA, navB), file=compareFD)
  print('</dl>', file=compareFD)
  print('<hr/><table id="SortThisTable" class="table table-striped table-hover table-bordered table-condensed">', file=compareFD)
  print('<thead><tr><th><span class="glyphicon glyphicon-folder-open"></span>&nbsp;H5 File</th>'+
        '<th><span class="glyphicon glyphicon-folder-close"></span>&nbsp;Dataset</th>'+
        '<th><span class="glyphicon glyphicon-search"></span>&nbsp;Label</th>'+
        '<th><span class="glyphicon glyphicon-search"></span>&nbsp;Result</th></tr></thead><tbody>', file=compareFD)

  nrAll=[0]
  nrFailed=[0]
  try:
    gnuplotProcess=subprocess.Popen(["gnuplot"], stdin=subprocess.PIPE)
    gnuplotProcess.stdin.write(("set terminal svg size 900, 1400\n").encode("utf-8"))
    gnuplotProcess.stdin.write(("set xlabel 'Time'\n").encode("utf-8"))
  except OSError:
    gnuplotProcess=None
    print("gnuplot not found. Hence no compare plot will be generated. Add gnuplot to PATH to enable.")
  data=[]
  for h5RefFileName in glob.glob(pj("reference", "*.h5")):
    # open h5 files
    h5RefFile=h5py.File(h5RefFileName, "r")
    try:
      h5CurFile=h5py.File(h5RefFileName[10:], "r")
    except IOError:
      data.append([
        (h5RefFile.filename[10:],""),
        ("no such file in current solution","d"),
        ("failed","d"),
        ("failed","d")
      ])
      nrAll[0]+=1
      nrFailed[0]+=1
    else:
      # process h5 file
      refMemberNames=set()
      # bind arguments h5CurFile, data, example, nrAll, nrFailed in order (nrAll, nrFailed as lists to pass by reference)
      dummyFctPtr = functools.partial(compareDatasetVisitor, h5CurFile, data, example, nrAll,
                                      nrFailed, refMemberNames, gnuplotProcess)
      h5RefFile.visititems(dummyFctPtr) # visit all dataset
      # check for datasets in current but not in reference
      curMemberNames=set()
      h5CurFile.visititems(functools.partial(appendDatasetName, curMemberNames)) # get all dataset names in cur
      for datasetName in curMemberNames-refMemberNames:
        data.append([
          (h5CurFile.filename,""),
          (datasetName,""),
          ('not in ref. but in cur.',"d"),
          ('failed',"d")
        ])
        nrAll[0]+=1
        nrFailed[0]+=1
      # close h5 files
      h5RefFile.close()
      h5CurFile.close()
  if gnuplotProcess!=None:
    gnuplotProcess.stdin.close()
    gnuplotProcess.wait()
  # files in current but not in reference
  refFiles=glob.glob(pj("reference", "*.h5"))
  for curFile in glob.glob("*.h5"):
    if pj("reference", curFile) not in refFiles:
      data.append([
        (curFile,""),
        ('no such file in reference solution',"d"),
        ('failed',"d"),
        ('failed',"d")
      ])
      nrAll[0]+=1
      nrFailed[0]+=1

  # all table data is now collected in data: print it now as javascript code using by DataTables in the browser
  print('<script type="text/javascript">', file=compareFD)
  print('var SortThisTable_data=[', file=compareFD)
  for row in data:
    print('{', end="", file=compareFD)
    for i in range(0,4):
      cont=row[i][0].replace("'", "\\'")
      if i==0 or i==1:
        cont=cont.replace('/', u'/\u200B')
      print("'d%d':'%s',"%(i, cont), end="", file=compareFD) # d<colIndex> == data for column <colIndex>
    for i in range(2,4):
      print("'c%d':'%s',"%(i, row[i][1]), end="", file=compareFD) # c<colIndex> == "", "d", "w" or "s" flag for column <colIndex>
    print('},', file=compareFD)
  print('];', file=compareFD)
  print('</script>', file=compareFD)

  # print html footer
  print('</tbody></table>', file=compareFD)
  print('<hr/>', file=compareFD)
  print('<p class="text-right small">', file=compareFD)
  print('  <a href="http://validator.w3.org/check?uri=referer">', file=compareFD)
  print('    <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>', file=compareFD)
  print('  </a>', file=compareFD)
  print('  Generated on %s by runexamples.py'%(str(timeID)), file=compareFD)
  print('</p>', file=compareFD)
  print('</body>', file=compareFD)
  print('</html>', file=compareFD)

  compareFD.close()
  return 1 if nrFailed[0]>0 else 0, nrFailed[0], nrAll[0]



def loopOverReferenceFiles(msg, srcPostfix, dstPrefix, action):
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    # remove dst dir and recreate it empty
    if os.path.isdir(pj(dstPrefix, example[0], "reference")): shutil.rmtree(pj(dstPrefix, example[0], "reference"))
    os.makedirs(pj(dstPrefix, example[0], "reference"))
    # loop over src
    curNumber+=1
    print("%s: Example %03d/%03d; %5.1f%%; %s"%(msg, curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    if not os.path.isdir(pj(dstPrefix, example[0], "reference")): os.makedirs(pj(dstPrefix, example[0], "reference"))
    for h5File in glob.glob(pj(example[0], srcPostfix, "*.h5")):
      action(h5File, pj(dstPrefix, example[0], "reference", os.path.basename(h5File)))
    if os.path.isfile(pj(example[0], srcPostfix, "time.dat")):
      action(pj(example[0], srcPostfix, "time.dat"), pj(dstPrefix, example[0], "reference", "time.dat"))

def copyToReference():
  loopOverReferenceFiles("Copy to reference", ".", ".", shutil.copyfile)

def copyAndSHA1AndAppendIndex(src, dst):
  # copy src to dst
  shutil.copyfile(src, dst)
  # create sha1 hash of dst (save to <dst>.sha1)
  codecs.open(dst+".sha1", "w", encoding="utf-8").write(hashlib.sha1(codecs.open(dst, "rb").read()).hexdigest())
  # add file to index
  index=codecs.open(pj(os.path.dirname(dst), "index.txt"), "a", encoding="utf-8")
  index.write(os.path.basename(dst)+":")
def pushReference():
  loopOverReferenceFiles("Pushing reference to download dir", "reference", args.pushDIR, copyAndSHA1AndAppendIndex)

def downloadFileIfDifferent(src):
  remoteSHA1Url=args.updateURL+"/"+myurllib.pathname2url(src+".sha1")
  remoteSHA1=myurllib.urlopen(remoteSHA1Url).read().decode('utf-8')
  try:
    localSHA1=hashlib.sha1(codecs.open(src, "rb").read()).hexdigest()
  except IOError: 
    localSHA1=""
  if remoteSHA1!=localSHA1:
    remoteUrl=args.updateURL+"/"+myurllib.pathname2url(src)
    print("  Download "+remoteUrl)
    if not os.path.isdir(os.path.dirname(src)): os.makedirs(os.path.dirname(src))
    codecs.open(src, "wb").write(myurllib.urlopen(remoteUrl).read())
def updateReference():
  curNumber=0
  lenDirs=len(directories)
  for example in directories:
    # print message
    curNumber+=1
    print("%s: Example %03d/%03d; %5.1f%%; %s"%("Update reference", curNumber, lenDirs, curNumber/lenDirs*100, example[0]))
    # loop over all file in src/index.txt
    indexUrl=args.updateURL+"/"+myurllib.pathname2url(pj(example[0], "reference", "index.txt"))
    try:
      for fileName in myurllib.urlopen(indexUrl).read().decode("utf-8").rstrip(":").split(":"):
        downloadFileIfDifferent(pj(example[0], "reference", fileName))
    except (IOError):
      pass



def listExamples():
  print('The following examples will be run:\n')
  for example in directories:
    print(example[0])



def validateXML(example, consoleOutput, htmlOutputFD):
  nrFailed=0
  nrTotal=0
  types=[["*.ombv.xml",                ombvSchema], # validate openmbv files generated by MBSim
         ["*.ombv.env.xml",            ombvSchema], # validate openmbv environment user files
         ["MBS.mbsimprj.flat.xml", mbsimXMLSchema]] # validate user mbsim flat xml files
  for root, _, filenames in os.walk(os.curdir):
    for curType in types:
      for filename in fnmatch.filter(filenames, curType[0]):
        outputFN=pj(example[0], filename+".txt")
        outputFD=MultiFile(codecs.open(pj(args.reportOutDir, outputFN), "w", encoding="utf-8"), args.printToConsole)
        print('<tr>', file=htmlOutputFD)
        print('<td>'+filename.replace('/', u'/\u200B')+'</td>', file=htmlOutputFD)
        print("Running command:", file=outputFD)
        list(map(lambda x: print(x, end=" ", file=outputFD), [mbxmlutilsvalidate, curType[1], pj(root, filename)]))
        print("\n", file=outputFD)
        outputFD.flush()
        if subprocessCall([mbxmlutilsvalidate, curType[1], pj(root, filename)],
                          outputFD)!=0:
          nrFailed+=1
          print('<td class="danger"><span class="glyphicon glyphicon-exclamation-sign alert-danger"></span>&nbsp;<a href="'+myurllib.pathname2url(filename+".txt")+'">failed</a></td>', file=htmlOutputFD)
        else:
          print('<td class="success"><span class="glyphicon glyphicon-ok-sign alert-success"></span>&nbsp;<a href="'+myurllib.pathname2url(filename+".txt")+'">passed</a></td>', file=htmlOutputFD)
        print('</tr>', file=htmlOutputFD)
        nrTotal+=1
        outputFD.close()
  return nrFailed, nrTotal



def writeRSSFeed(nrFailed, nrTotal):
  rssFN="result.rss.xml"
  rssFD=codecs.open(pj(args.reportOutDir, os.pardir, rssFN), "w", encoding="utf-8")
  print('''\
<?xml version="1.0" encoding="UTF-8"?>
<rss version="2.0" xmlns:atom="http://www.w3.org/2005/Atom">
  <channel>
    <title>%sMBSim runexample.py Result</title>
    <link>%s/result_current/index.html</link>
    <description>%sResult RSS feed of the last runexample.py run of MBSim and Co.</description>
    <language>en-us</language>
    <managingEditor>friedrich.at.gc@googlemail.com (friedrich)</managingEditor>
    <atom:link href="%s/%s" rel="self" type="application/rss+xml"/>'''%(args.buildType, args.url, args.buildType, args.url, rssFN), file=rssFD)
  if nrFailed>0:
    currentID=int(os.path.basename(args.reportOutDir)[len("result_"):])
    print('''\
    <item>
      <title>%s%d of %d examples failed</title>
      <link>%s/result_%010d/index.html</link>
      <guid isPermaLink="false">%s/result_%010d/rss_id_%s</guid>
      <pubDate>%s</pubDate>
    </item>'''%(args.buildType, nrFailed, nrTotal,
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



#####################################################################################
# call the main routine
#####################################################################################

if __name__=="__main__":
  mainRet=main()
  exit(mainRet)

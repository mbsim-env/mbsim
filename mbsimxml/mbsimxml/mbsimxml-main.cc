#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif
#include "config.h"
#include <clocale>
#include <cfenv>
#include <cassert>
#include <iostream>
#include <regex>
#include <list>
#include <vector>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <boost/dll.hpp>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/windows_signal_conversion.h>
#include "mbxmlutils/preprocess.h"
#include <mbsim/element.h>
#include <mbsim/integrators/integrator.h>
#include <mbsim/dynamic_system_solver.h>
#include <cstdio>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include "mbsimxml.h"
#include "mbsimflatxml.h"
#include <openmbvcppinterface/objectfactory.h>
#include "set_current_path.h"
#include <condition_variable>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

using namespace std;
using namespace MBXMLUtils;
using namespace MBSim;
namespace bfs=boost::filesystem;

namespace MBSim {
  extern int baseIndexForPlot;
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  SetConsoleCP(CP_UTF8);
  SetConsoleOutputCP(CP_UTF8);
#endif
  MBXMLUtils::handleFPE();
  setlocale(LC_ALL, "C");
  MBXMLUtils::convertWMCLOSEtoSIGTERM();

  try {
    // check for errors during ObjectFactory
    string errorMsg(OpenMBV::ObjectFactory::getAndClearErrorMsg());
    if(!errorMsg.empty()) {
      cerr<<"The following errors occured during the pre-main code of the OpenMBVC++Interface object factory:"<<endl;
      cerr<<errorMsg;
      cerr<<"Exiting now."<<endl;
      return 1;
    }

    // check for errors during ObjectFactory
    string errorMsg2(ObjectFactory::getAndClearErrorMsg());
    if(!errorMsg2.empty()) {
      cerr<<"The following errors occured during the pre-main code of the MBSim object factory:"<<endl;
      cerr<<errorMsg2;
      cerr<<"Exiting now."<<endl;
      return 1;
    }

    // convert args to c++
    list<string> args;
    list<string>::iterator i, i2;
    for(int i=1; i<argc; i++)
      args.emplace_back(argv[i]);

    // help
    if(args.size()<1 ||
       std::find(args.begin(), args.end(), "-h")!=args.end() ||
       std::find(args.begin(), args.end(), "--help")!=args.end() ||
       std::find(args.begin(), args.end(), "-?")!=args.end()) {
      cout<<"Usage: mbsimxml [--onlypreprocess|--donotintegrate|--stopafterfirststep|"<<endl
          <<"                 --autoreload [ms]|--dumpXMLCatalog <file>] [--savefinalstatevector]"<<endl
          <<"                [--modulePath <dir> [--modulePath <dir> ...]]"<<endl
          <<"                [--stdout <msg> [--stdout <msg> ...]] [--stderr <msg> [--stderr <msg> ...]]"<<endl
          <<"                [<paramname>=<value> [<paramname>=<value> ...]]"<<endl
          <<"                [-C <dir/file>|--CC] [--onlyLatestStdin] <mbsimprjfile>|-"<<endl
          <<""<<endl
          <<"Copyright (C) 2004-2009 MBSim Development Team"<<endl
          <<"This is free software; see the source for copying conditions. There is NO"<<endl
          <<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl
          <<""<<endl
          <<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl
          <<""<<endl
          <<"--onlypreprocess         Stop after the preprocessing stage"<<endl
          <<"--donotintegrate         Stop after the initialization stage, do not integrate"<<endl
          <<"--savefinalstatevector   Save the state vector to the file \"statevector.asc\" after integration"<<endl
          <<"--savestatetable         Save the state table to the file \"statetable.asc\""<<endl
          <<"--stopafterfirststep     Stop after outputting the first step (usually at t=0)"<<endl
          <<"                         This generates a HDF5 output file with only one time serie"<<endl
          <<"--autoreload             Same as --stopafterfirststep but rerun mbsimxml each time"<<endl
          <<"                         a input file is newer than the output file."<<endl
          <<"                         Checked every ms. Ignored if '-' is given."<<endl
          <<"--dumpXMLCatalog <file>  Dump a XML catalog for all schemas including MBSim modules to file and exit"<<endl
          <<"--modulePath <dir>       Add <dir> to MBSim module serach path. The central MBSim installation"<<endl
          <<"                         module dir and the current dir is always included."<<endl
          <<"                         Also added are all directories listed in the file"<<endl
          <<"                         Linux: $XDG_CONFIG_HOME/mbsim-env/mbsimxml.modulepath"<<endl
          <<"                         Windows: %APPDATA%\\mbsim-env\\mbsimxml.modulepath"<<endl
          <<"                         This file contains one directory per line."<<endl
          <<"<paramname>=<value>      Override the MBSimProject parameter named <paramname> with <value>."<<endl
          <<"                         <value> is evaluated using the evaluator defined in MBSimProject"<<endl
          <<"--stdout <msg>           Print on stdout messages of type <msg>."<<endl
          <<"                         <msg> may be info~<pre>~<post>, warn~<pre>~<post>, debug~<pre>~<post>"<<endl
          <<"                         error~<pre>~<post>~ or depr~<pre>~<post>~."<<endl
          <<"                         Each message is prefixed/postfixed with <pre>/<post>."<<endl
          <<"                         --stdout may be specified multiple times."<<endl
          <<"--stderr <msg>           Analog to --stdout but prints to stderr."<<endl
          <<"-C <dir/file>            Change current to dir to <dir>/dir of <file> first."<<endl
          <<"                         All arguments are still relative to the original current dir."<<endl
          <<"--CC                     Change current dir to dir of <mbsimprjfile> first. Ignored if '-' is given."<<endl
          <<"                         All arguments are still relative to the original current dir."<<endl
          <<"<mbsimprjfile>           Use <mbsimprjfile> as mbsim xml project file (write output to current working dir)"<<endl
          <<"                         Must be the last argument!"<<endl
          <<"-                        Read project file from stdin, which must end with a null-char, and process it."<<endl
          <<"                         Then the next project file is read or the program ends if EOF is reached."<<endl
          <<"                         If the input contains relative references to other files then the root element of the input"<<endl
          <<"                         must be tagged using a 'MBXMLUtils_OriginalFilename' XML-Processing-Instruction element."<<endl
          <<"                         Must be the last argument!"<<endl
          <<"--onlyLatestStdin        If '-' and --onlyLatestStdin is used, inputs on stdin are skipped if already"<<endl
          <<"                         a newer input is waiting on the input buffer and the currently running input"<<endl
          <<"                         is interrupted silently before starting the new input."<<endl;
      return 0;
    }

    // handle --stdout and --stderr args
    setupMessageStreams(args);
  
    // parse parameters

    // current directory and MBSIMPRJ
    bfs::path MBSIMPRJ=*(--args.end());
    bfs::path newCurrentPath;
    if(MBSIMPRJ!="-" && (i=std::find(args.begin(), args.end(), "--CC"))!=args.end()) {
      newCurrentPath=MBSIMPRJ.parent_path();
      args.erase(i);
    }
    if((i=std::find(args.begin(), args.end(), "-C"))!=args.end()) {
      i2=i; i2++;
      if(bfs::is_directory(*i2))
        newCurrentPath=*i2;
      else
        newCurrentPath=bfs::path(*i2).parent_path();
      args.erase(i);
      args.erase(i2);
    }
    SetCurrentPath currentPath(newCurrentPath);
    // MBSIMPRJ=currentPath.adaptPath(MBSIMPRJ); delayed, see call below
    for(auto a : {"--modulePath", "--dumpXMLCatalog"})
      if(auto i=std::find(args.begin(), args.end(), a); i!=args.end()) {
        auto i2=i; i2++;
        *i2=currentPath.adaptPath(*i2).string();
      }


    // generate mbsimxml.xsd
    set<bfs::path> searchDirs;
    while((i=find(args.begin(), args.end(), "--modulePath"))!=args.end()) {
      i2=i; i2++;
      searchDirs.insert(*i2);
      args.erase(i);
      args.erase(i2);
    }

    // create xml catalog
    auto xmlCatalogDoc=MBSim::getMBSimXMLCatalog(searchDirs);

    if((i=std::find(args.begin(), args.end(), "--dumpXMLCatalog"))!=args.end()) {
      i2=i; i2++;
      if(i2==args.end())
      {
        cerr<<"No filename specified after --dumpXMLCatalog."<<endl;
        return 1;
      }
      DOMParser::serialize(xmlCatalogDoc->getDocumentElement(), *i2);
      args.erase(i2);
      args.erase(i);
      return 0;
    }

    bool ONLYPP=false;
    if((i=std::find(args.begin(), args.end(), "--onlypreprocess"))!=args.end()) {
      ONLYPP=true;
      args.erase(i);
    }
  
    bool doNotIntegrate=false;
    if((i=std::find(args.begin(), args.end(), "--donotintegrate"))!=args.end()) {
      doNotIntegrate=true;
      args.erase(i);
    }
  
    bool stopAfterFirstStep=false;
    if((i=std::find(args.begin(), args.end(), "--stopafterfirststep"))!=args.end()) {
      stopAfterFirstStep=true;
      args.erase(i);
    }

    bool savestatetable=false;
    if((i=std::find(args.begin(), args.end(), "--savestatetable"))!=args.end()) {
      savestatetable=true;
      args.erase(i);
    }

    bool savestatevector=false;
    if((i=std::find(args.begin(), args.end(), "--savefinalstatevector"))!=args.end()) {
      savestatevector=true;
      args.erase(i);
    }

    int AUTORELOADTIME=0;
    if(MBSIMPRJ!="-" && (i=std::find(args.begin(), args.end(), "--autoreload"))!=args.end()) {
      i2=i; i2++;
      char *error;
      stopAfterFirstStep=true;
      AUTORELOADTIME=strtol(i2->c_str(), &error, 10);
      // AUTORELOAD is set delayed since we currently do not know the MBSim file name (see later)
      if(AUTORELOADTIME<0) AUTORELOADTIME=250;
      args.erase(i);
      if(error && strlen(error)==0)
        args.erase(i2);
      else
        AUTORELOADTIME=250;
    }

    // parameter overrides
    regex paramRE("^[_a-zA-Z][_a-zA-Z0-9]*=");
    set<string> paramArg;
    while((i=find_if(args.begin(), args.end(), [&paramRE](auto &p){ return regex_search(p, paramRE); })) != args.end()) {
      paramArg.emplace(*i);
      args.erase(i);
    }

    bool onlyLatestStdin=false;
    if((i=std::find(args.begin(), args.end(), "--onlyLatestStdin"))!=args.end()) {
      onlyLatestStdin=true;
      args.erase(i);
    }

    args.pop_back();
    if(MBSIMPRJ!="-")
      MBSIMPRJ=currentPath.adaptPath(MBSIMPRJ);
  
    if(args.size()!=0) {
      cerr<<"Unhandled arguments found:"<<endl;
      for(auto &x : args)
        cerr<<x<<" ";
      cerr<<endl;
      return 1;
    }

    // create parser from catalog file to avoid regenerating the parser if multiple inputs are handled
    auto parser = DOMParser::create(xmlCatalogDoc->getDocumentElement());

    thread stdinReadThread;
    mutex mu;
    condition_variable cv;
    optional<string> lastStdinContent; // if set its the content to be processed
    bool cinEOF=false; // if true stdin has reached EOF and the program should exit
    int stdinFDDup = -1;
    if(MBSIMPRJ=="-" && onlyLatestStdin) {
      // Some evaluators (e.g. Python) use stdin e.g. during initialization (setting flags on stdin, ...).
      // Hence, when we read on stdin (=cin) in a thread this may cause race-conditions with evaluator code.
      // To avoid this we close stdin (=fd 0). This will cause at least Python not to to anything on stdin.
      // Before closing stdin we duplicate it and use this duplicate fd instead of stdin.
      // And use boost::iostreams to get a c++ stream which operators on the duplicated fd.
      stdinFDDup=dup(fileno(stdin));
      if(stdinFDDup==-1)
        throw runtime_error("Failed to duplicate the stdin FD.");
      close(fileno(stdin));
      // cinDup is created in side the thread just for lifetime issues

      stdinReadThread = thread([&mu, &cv, &cinEOF, &lastStdinContent, stdinFDDup](){
        boost::iostreams::filtering_istream cinDup;
        cinDup.push(boost::iostreams::file_descriptor_source(stdinFDDup, boost::iostreams::never_close_handle));

        while(true) { // read from stdin in a endless loop
          string str;
          getline(cinDup, str, '\0'); // read content (blocking read but not locked by "mu"
          // now lock "mu" ...
          lock_guard l(mu);
          cv.notify_all(); // ... notify about the new information which is ...
          if(cinDup.eof()) { // ... EOF reached ...
            cinEOF=true;
            break;
          }
          lastStdinContent=str; // ... or set lastStdinContent to the read content
          DynamicSystemSolver::interrupt(true); // silently interrupt a already running simulation (we want always to run the latest input only)
        };
      });
    }

    // execute
    int ret; // command return value
    bool runAgain=true; // always run the first time
    while(runAgain) {
      ret=0;

      vector<bfs::path> dependencies;

      try {
        // run preprocessor

        unique_ptr<DynamicSystemSolver::SignalHandler> sigHandler; // install signal handler from now on (and deinstall on scope exit)

        stringstream MBSIMPRJstream;
        if(MBSIMPRJ=="-") {
          string str;
          if(onlyLatestStdin) {
            // lock "mu" and check in a endless loop for new information from the thread
            unique_lock l(mu);
            while(true) {
              if(cinEOF) // if EOF has reached break this loop and the also the outer loop
                break;
              if(lastStdinContent) { // if lastStdinContent is set, use this content and reset lastStdinContent
                str=lastStdinContent.value();
                lastStdinContent.reset();
                break;
              }
              // now wait for notification from the thread to avoid CPU load in this endless loop
              cv.wait(l);
            }
            if(cinEOF) // if EOF has reached break this outer loop which exits this program
              break;
            sigHandler=make_unique<DynamicSystemSolver::SignalHandler>(); // this must be called inside of the lock of mu [unique_lock l(mu)]
          }
          else {
            getline(cin, str, '\0'); // read the next file content (up to next null-char or EOF)
            if(cin.eof())
              break;
            sigHandler=make_unique<DynamicSystemSolver::SignalHandler>();
          }
          MBSIMPRJstream.str(std::move(str)); // this warning will be gone with c++20
        }

        Preprocess preprocess = MBSIMPRJ=="-" ?
          Preprocess(MBSIMPRJstream, parser, AUTORELOADTIME>0) : // ctor for input by stdin
          Preprocess(MBSIMPRJ, parser, AUTORELOADTIME>0);        // ctor for input by filename
        preprocess.setCheckInterruptFunction([](){
          DynamicSystemSolver::throwIfExitRequested();
        });

        // check Embed elements
        {
          auto checkEmbed = [](xercesc::DOMElement *e, const FQN &eleName, bool allowHref) {
            if(E(e)->getTagName()==PV%"Embed") {
              if(E(e)->hasAttribute("counterName") || E(e)->hasAttribute("count") ||
                 (E(e)->hasAttribute("onlyif") && E(e)->getAttribute("onlyif")!="1"))
                throw runtime_error("A Embed element on "+eleName.second+" level is not allowed to have a counterName, count or onlyif attribute.");
              if(!allowHref && E(e)->hasAttribute("href"))
                throw runtime_error("A Embed element on "+eleName.second+" level is not allowed to have a href attribute.");
            }
          };

          auto root = preprocess.getDOMDocument()->getDocumentElement();
          xercesc::DOMElement *mbsimProject;
          checkEmbed(root, PV%"MBSimProject", false);
          if(E(root)->getTagName()==PV%"Embed")
            mbsimProject = root->getLastElementChild();
          else
            mbsimProject = root;
          checkEmbed(mbsimProject->getFirstElementChild(), MBSIM%"DynamicSystemSolver", true);
          checkEmbed(mbsimProject->getLastElementChild(), MBSIM%"Solver", true);
        }

        // create parameter override ParamSet
        auto eval=preprocess.getEvaluator();
        auto param = make_shared<Preprocess::ParamSet>();
        for(auto &pa : paramArg) {
          DynamicSystemSolver::throwIfExitRequested();
          auto pos = pa.find('=');
          (*param)[pa.substr(0, pos)]=eval->eval(pa.substr(pos+1));
        }
        auto overrideParam(*param);
        preprocess.setParam(param);

        // validate the project file with mbsimxml.xsd
        auto mainXMLDoc = preprocess.processAndGetDocument();

        if(!ONLYPP) {
          // load MBSim modules
          MBSimXML::loadModules(searchDirs);
          // check for errors during ObjectFactory
          string errorMsg3(ObjectFactory::getAndClearErrorMsg());
          if(!errorMsg3.empty()) {
            cerr<<"The following errors occured during the loading of MBSim modules object factory:"<<endl;
            cerr<<errorMsg3;
            cerr<<"Exiting now."<<endl;
            return 1;
          }

          auto e=mainXMLDoc->getDocumentElement();
          // create object for DynamicSystemSolver and check correct type
          e=E(e)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver");
          fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Instantiate DynamicSystemSolver"<<endl;
          auto dss=unique_ptr<DynamicSystemSolver>(ObjectFactory::createAndInit<DynamicSystemSolver>(e));
        
          // create object for Solver and check correct type
          fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Instantiate Solver"<<endl;
          auto solver=unique_ptr<Solver>(ObjectFactory::createAndInit<Solver>(e->getNextElementSibling()));

          // init dss
          if(doNotIntegrate)
            dss->setTruncateSimulationFiles(false);
          dss->initialize();

          MBSimXML::main(solver, dss, doNotIntegrate, stopAfterFirstStep, savestatevector, savestatetable);
        }

        if(AUTORELOADTIME>0)
          dependencies = preprocess.getDependencies();
      }
      catch(const SilentError &) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<flush<<skipws<<"Exception due to silent user requested exit."<<flush<<noskipws<<endl;
      }
      catch(const MBSimError &ex) {
        // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<ex.what()<<flush<<noskipws<<endl;
        ret=1;
      }
      catch(const DOMEvalException &ex) {
        // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<ex.what()<<flush<<noskipws<<endl;
        ret=1;
      }
      catch(const exception &ex) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<ex.what()<<endl;
        ret=1;
      }
      catch(...) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception"<<endl;
        ret=1;
      }

      if(MBSIMPRJ!="-") {
        runAgain=false; // only run ones except --autoreload is given

        if(AUTORELOADTIME>0) {
          boost::posix_time::ptime lastRun=boost::posix_time::microsec_clock::universal_time();
    
          // check for dependent files being newer then the output file
          while(!runAgain) {
#ifndef _WIN32
            usleep(AUTORELOADTIME*1000);
#else
            Sleep(AUTORELOADTIME);
#endif
            dependencies.emplace_back(MBSIMPRJ);
            for(const auto & depfile : dependencies) {
              if(boost::myfilesystem::last_write_time(depfile.string())>lastRun) {
                runAgain=true;
                break;
              }
            }
          }
        }
      }
      else
        runAgain=true;
    }
    if(stdinReadThread.joinable()) {
      if(dup2(stdinFDDup, 0)==-1) // revert the stdin FD duplicate (not needed by a nice cleanup)
        throw runtime_error("Failed to re-duplicate the stdin FD.");
      stdinReadThread.join();// wait until the thread has finished
    }
  
    return ret;
  }
  catch(const MBSimError &e) {
    // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<e.what()<<flush<<noskipws<<endl;
    return 1;
  }
  catch(const DOMEvalException &e) {
    // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<e.what()<<flush<<noskipws<<endl;
    return 1;
  }
  catch(const exception &e) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<e.what()<<endl;
    return 1;
  }
  catch(...) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception"<<endl;
    return 1;
  }
  return 0;
}

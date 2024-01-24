#include "config.h"
#include <clocale>
#include <cfenv>
#include <iostream>
#include <regex>
#include <list>
#include <vector>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <boost/dll.hpp>
#include <mbxmlutilshelper/last_write_time.h>
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
  setlocale(LC_ALL, "ACP.UTF-8");
#else
  //assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1); // Qt seems to generate some FPE, hence not activated  
  setlocale(LC_ALL, "C");
#endif

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
          <<"                [--baseindexforplot <bi>] [--modulePath <dir> [--modulePath <dir> ...]]"<<endl
          <<"                [--stdout <msg> [--stdout <msg> ...]] [--stderr <msg> [--stderr <msg> ...]]"<<endl
          <<"                [<paramname>=<value> [<paramname>=<value> ...]]"<<endl
          <<"                [-C <dir/file>|--CC] <mbsimprjfile>"<<endl
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
	  <<"--baseindexforplot <bi>  Use the base index <bi> in MBSim plot files"<<endl
          <<"--stopafterfirststep     Stop after outputting the first step (usually at t=0)"<<endl
          <<"                         This generates a HDF5 output file with only one time serie"<<endl
          <<"--autoreload             Same as --stopafterfirststep but rerun mbsimxml each time"<<endl
          <<"                         a input file is newer than the output file. Checked every ms."<<endl
          <<"--dumpXMLCatalog <file>  Dump a XML catalog for all schemas including MBSim modules to file and exit"<<endl
          <<"--modulePath <dir>       Add <dir> to MBSim module serach path. The central MBSim installation"<<endl
          <<"                         module dir and the current dir is always included."<<endl
          <<"                         Also added are all directories listed in the file"<<endl
          <<"                         Linux: $HOME/.config/mbsim-env/mbsimxml.modulepath"<<endl
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
          <<"--CC                     Change current dir to dir of <mbsimprjfile> first."<<endl
          <<"                         All arguments are still relative to the original current dir."<<endl
          <<"<mbsimprjfile>           Use <mbsimprjfile> as mbsim xml project file (write output to current working dir)"<<endl
          <<"                         Must be the last argument!"<<endl;
      return 0;
    }

    // handle --stdout and --stderr args
    setupMessageStreams(args);
  
    // get path of this executable
    string EXEEXT;
#ifdef _WIN32 // Windows
    EXEEXT=".exe";
#else // Linux
    EXEEXT="";
#endif
  
    bfs::path MBXMLUTILSBIN=boost::dll::program_location().parent_path().parent_path()/"bin";
    bfs::path MBXMLUTILSSCHEMA=boost::dll::program_location().parent_path().parent_path()/"share"/"mbxmlutils"/"schema";
  
    // parse parameters

    // current directory and MBSIMPRJ
    bfs::path MBSIMPRJ=*(--args.end());
    bfs::path newCurrentPath;
    if((i=std::find(args.begin(), args.end(), "--CC"))!=args.end()) {
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

    if((i=std::find(args.begin(), args.end(), "--baseindexforplot"))!=args.end()) {
      i2=i; i2++;
      MBSim::baseIndexForPlot=stoi(*i2);
      args.erase(i);
      args.erase(i2);
    }

    int AUTORELOADTIME=0;
    if((i=std::find(args.begin(), args.end(), "--autoreload"))!=args.end()) {
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

    args.pop_back();
    MBSIMPRJ=currentPath.adaptPath(MBSIMPRJ);
  
    if(args.size()!=0) {
      cerr<<"Unhandled arguments found:"<<endl;
      for(auto &x : args)
        cerr<<x<<" ";
      cerr<<endl;
      return 1;
    }
  
    // execute
    int ret; // comamnd return value
    bool runAgain=true; // always run the first time
    while(runAgain) {
      ret=0;

      vector<bfs::path> dependencies;

      try {
        // run preprocessor

        // validate the project file with mbsimxml.xsd
        auto [mainXMLDoc, evalName] = Preprocess::parseFileAndGetEvaluator(dependencies, xmlCatalogDoc->getDocumentElement(), MBSIMPRJ);

        // create parameter override ParamSet
        auto eval=Eval::createEvaluator(evalName, &dependencies);
        auto param = make_shared<Preprocess::ParamSet>();
        for(auto &pa : paramArg) {
          auto pos = pa.find('=');
          (*param)[pa.substr(0, pos)]=eval->eval(pa.substr(pos+1));
        }
        auto overrideParam(*param);

        // validate the project file with mbsimxml.xsd
        Preprocess::preprocessFile(dependencies, eval, mainXMLDoc, param);

        // print parameter overrides
        for(auto &p : overrideParam) {
          auto it = param->find(p.first);
          if(it != param->end())
            fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Parameter '"<<p.first<<"' overwritten with value "<<eval->cast<CodeString>(p.second)<<endl;
          else
            fmatvec::Atom::msgStatic(fmatvec::Atom::Warn)<<"Parameter '"<<p.first<<"' not found and not overwritten"<<endl;
        }

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
          auto dss=unique_ptr<DynamicSystemSolver>(ObjectFactory::createAndInit<DynamicSystemSolver>(e));
        
          // create object for Solver and check correct type
          auto solver=unique_ptr<Solver>(ObjectFactory::createAndInit<Solver>(e->getNextElementSibling()));

          // init dss
          if(doNotIntegrate)
            dss->setTruncateSimulationFiles(false);
          dss->initialize();

          MBSimXML::main(solver, dss, doNotIntegrate, stopAfterFirstStep, savestatevector, savestatetable);
        }
      }
      catch(const exception &ex) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<ex.what()<<endl;
        ret=1;
      }
      catch(...) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception"<<endl;
        ret=1;
      }
      boost::posix_time::ptime lastRun=boost::posix_time::microsec_clock::universal_time();
  
      runAgain=false; // only run ones except --autoreload is given
      if(AUTORELOADTIME>0) {
        // check for dependent files being newer then the output file
        while(!runAgain) {
#ifndef _WIN32
          usleep(AUTORELOADTIME*1000);
#else
          Sleep(AUTORELOADTIME);
#endif
          for(const auto & depfile : dependencies) {
            if(boost::myfilesystem::last_write_time(depfile.string())>lastRun) {
              runAgain=true;
              break;
            }
          }
        }
      }
    }
  
    return ret;
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

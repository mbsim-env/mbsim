#include "config.h"
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

using namespace std;
using namespace MBXMLUtils;
using namespace MBSim;
namespace bfs=boost::filesystem;

int main(int argc, char *argv[]) {
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

    bool ONLYLISTSCHEMAS=std::find(args.begin(), args.end(), "--onlyListSchemas")!=args.end();
  
    // help
    if(args.size()<1 ||
       std::find(args.begin(), args.end(), "-h")!=args.end() ||
       std::find(args.begin(), args.end(), "--help")!=args.end() ||
       std::find(args.begin(), args.end(), "-?")!=args.end()) {
      cout<<"Usage: mbsimxml [--onlypreprocess|--donotintegrate|--stopafterfirststep|"<<endl
          <<"                 --autoreload [ms]|--onlyListSchemas] [--savefinalstatevector]"<<endl
          <<"                [--modulePath <dir> [--modulePath <dir> ...]]"<<endl
          <<"                [--stdout <msg> [--stdout <msg> ...]] [--stderr <msg> [--stderr <msg> ...]]"<<endl
          <<"                <mbsimprjfile>"<<endl
          <<""<<endl
          <<"Copyright (C) 2004-2009 MBSim Development Team"<<endl
          <<"This is free software; see the source for copying conditions. There is NO"<<endl
          <<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl
          <<""<<endl
          <<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl
          <<""<<endl
          <<"--onlypreprocess       Stop after the preprocessing stage"<<endl
          <<"--donotintegrate       Stop after the initialization stage, do not integrate"<<endl
          <<"--savefinalstatevector Save the state vector to the file \"statevector.asc\" after integration"<<endl
          <<"--savestatetable       Save the state table to the file \"statetable.asc\""<<endl
          <<"--stopafterfirststep   Stop after outputting the first step (usually at t=0)"<<endl
          <<"                       This generates a HDF5 output file with only one time serie"<<endl
          <<"--autoreload           Same as --stopafterfirststep but rerun mbsimxml each time"<<endl
          <<"                       a input file is newer than the output file. Checked every ms."<<endl
          <<"--onlyListSchemas      List all XML schema files including MBSim modules"<<endl
          <<"--modulePath <dir>     Add <dir> to MBSim module serach path. The central MBSim installation"<<endl
          <<"                       module dir and the current dir is always included."<<endl
          <<"                       Also added are all directories listed in the file"<<endl
          <<"                       Linux: $HOME/.config/mbsim-env/mbsimxml.modulepath"<<endl
          <<"                       Windows: %APPDATA%\\mbsim-env\\mbsimxml.modulepath"<<endl
          <<"                       This file contains one directory per line."<<endl
          <<"--stdout <msg>         Print on stdout messages of type <msg>."<<endl
          <<"                       <msg> may be info~<pre>~<post>, warn~<pre>~<post>, debug~<pre>~<post>"<<endl
          <<"                       error~<pre>~<post>~ or depr~<pre>~<post>~."<<endl
          <<"                       Each message is prefixed/postfixed with <pre>/<post>."<<endl
          <<"                       --stdout may be specified multiple times."<<endl
          <<"                       If --stdout and --stderr is not specified --stdout 'info~Info: ~'"<<endl
          <<"                       --stderr 'warn~Warn: ~' --stderr 'error~~' --stderr 'depr~Depr:~'"<<endl
          <<"                       --stderr 'status~~\\r' is used."<<endl
          <<"--stderr <msg>         Analog to --stdout but prints to stderr."<<endl
          <<"<mbsimprjfile>         Use <mbsimprjfile> as mbsim xml project file"<<endl;
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

    // generate mbsimxml.xsd
    set<bfs::path> searchDirs;
    while((i=find(args.begin(), args.end(), "--modulePath"))!=args.end()) {
      i2=i; i2++;
      searchDirs.insert(*i2);
      args.erase(i);
      args.erase(i2);
    }
    set<bfs::path> schemas=MBSim::getMBSimXMLSchemas(searchDirs, !ONLYLISTSCHEMAS);
    if(ONLYLISTSCHEMAS) {
      for(auto &schema: schemas)
        cout<<schema.string()<<endl;
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
  
    string MBSIMPRJ=*args.begin();
    args.erase(args.begin());
  
    // execute
    int ret; // comamnd return value
    bool runAgain=true; // always run the first time
    while(runAgain) {
      ret=0;

      vector<bfs::path> dependencies;
  
      try {
        // run preprocessor
        // validate the project file with mbsimxml.xsd
        auto mainXMLDoc=Preprocess::preprocessFile(dependencies, schemas, MBSIMPRJ);
  
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
        cerr<<"Error running model:"<<endl<<ex.what()<<endl;
        ret=1;
      }
      catch(...) {
        cerr<<"Unknown error running model."<<endl;
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

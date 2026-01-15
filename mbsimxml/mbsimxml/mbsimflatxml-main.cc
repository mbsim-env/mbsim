#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif
#include "config.h"
#include <clocale>
#include <cfenv>
#include <cassert>
#include <cstring>
#include <regex>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/solver.h"
#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/utils.h>
#include <mbxmlutilshelper/windows_signal_conversion.h>
#include <openmbvcppinterface/objectfactory.h>

using namespace std;
using namespace MBSim;

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
    DynamicSystemSolver::SignalHandler dummy; // install signal handler for next line (and deinstall on scope exit)

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

    list<string> args;
    list<string>::iterator i, i2;
    for(int i=1; i<argc; ++i)
      args.emplace_back(argv[i]);

    // handle --stdout and --stderr args
    MBXMLUtils::setupMessageStreams(args);
    bool doNotIntegrate=false;
    if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
      doNotIntegrate=true;
    bool stopAfterFirstStep=false;
    if(find(args.begin(), args.end(), "--stopafterfirststep")!=args.end())
      stopAfterFirstStep=true;
    bool savestatetable=false;
    if(find(args.begin(), args.end(), "--savestatetable")!=args.end())
      savestatetable=true;
    bool savestatevector=false;
    if(find(args.begin(), args.end(), "--savefinalstatevector")!=args.end())
      savestatevector=true;

    unique_ptr<Solver> solver;
    unique_ptr<DynamicSystemSolver> dss;
  
    if(MBSimXML::preInit(args, dss, solver)!=0) return 0; 
    if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
      dss->setTruncateSimulationFiles(false);
    dss->initialize();
  
    MBSimXML::main(solver, dss, doNotIntegrate, stopAfterFirstStep, savestatevector, savestatetable);
  }
  catch(const MBSimError &e) {
    // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<e.what()<<flush<<noskipws<<endl;
    return 1;
  }
  catch(const MBXMLUtils::DOMEvalException &e) {
    // DOMEvalException is already passed thought escapeFunc -> skip escapeFunc (if enabled on the fmatvec::Atom streams) from duing another escaping
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<flush<<skipws<<e.what()<<flush<<noskipws<<endl;
    return 1;
  }
  catch(const exception &e) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<e.what()<<endl;
    return 1;
  }
  catch(...) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception."<<endl;
    return 1;
  }

  return 0;
}

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  undef __STRICT_ANSI__ // to define _controlfp which is not part of ANSI and hence not defined in mingw
#  include <cfloat>
#  define __STRICT_ANSI__
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
  _controlfp(~(_EM_ZERODIVIDE | _EM_INVALID | _EM_OVERFLOW), _MCW_EM);
#else
  assert(feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW)!=-1);
#endif
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
    MBSimXML::initDynamicSystemSolver(args, dss);
  
    MBSimXML::main(solver, dss, doNotIntegrate, stopAfterFirstStep, savestatevector, savestatetable);
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

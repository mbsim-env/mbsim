#include "config.h"
#include <cstring>
#include <regex>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/solver.h"
#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbxmlutilshelper/utils.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {
    list<string> args;
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

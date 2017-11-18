#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/solver.h"
#include <boost/filesystem.hpp>
#include <boost/timer/timer.hpp>
#include <mbxmlutilshelper/last_write_time.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {
    DynamicSystemSolver::installSignalHandler();

    vector<string> args;
    for(int i=1; i<argc; ++i)
      args.emplace_back(argv[i]);

    Solver *solver;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(find(args.begin(), args.end(), "--donotintegrate")!=args.end())
      doNotIntegrate=true;
    bool stopAfterFirstStep=false;
    if(find(args.begin(), args.end(), "--stopafterfirststep")!=args.end())
      stopAfterFirstStep=true;
  
    if(MBSimXML::preInit(args, dss, solver)!=0) return 0; 
    MBSimXML::initDynamicSystemSolver(args, dss);
  
    if(doNotIntegrate==false) {
      if(stopAfterFirstStep)
        MBSimXML::plotInitialState(solver, dss);
      else {
        boost::timer::cpu_timer t;
        t.start();
        solver->execute(*dss);
        t.stop();
        cout<<"Integration CPU times: "<<t.format()<<endl;
      }
      // Remove the following block if --lastframe works in OpenMBV.
      // If this is removed openmbv should be opened with the --lastframe option.
      // Currently we use this block if --stopafterfirststep is given to reload the XML/H5 file in OpenMBV again
      // after the first step has been written since this is not possible by the file locking mechanism in OpenMBVCppInterface.
      if(stopAfterFirstStep) {
        // touch the OpenMBV files
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.xml").c_str(), boost::posix_time::microsec_clock::universal_time());
        boost::myfilesystem::last_write_time((dss->getName()+".ombv.h5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
      }
    }

    MBSimXML::postMain(args, solver, dss);
  }
  catch(const exception &e) {
    cerr<<"Exception:"<<endl
        <<e.what()<<endl;
    return 1;
  }
  catch(...) {
    cerr<<"Unknown exception"<<endl;
    return 1;
  }

  return 0;
}

#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/last_write_time.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {

    Solver *solver;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(argc>=2 && strcmp(argv[1],"--donotintegrate")==0)
      doNotIntegrate=true;
    bool stopAfterFirstStep=false;
    if(argc>=2 && strcmp(argv[1],"--stopafterfirststep")==0)
      stopAfterFirstStep=true;
  
    if(MBSimXML::preInit(argc, argv, dss, solver)!=0) return 0; 
    MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  
    if(doNotIntegrate==false) {
      if(stopAfterFirstStep)
        MBSimXML::plotInitialState(solver, dss);
      else
        MBSimXML::main(solver, dss);
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

    MBSimXML::postMain(argc, argv, solver, dss);
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

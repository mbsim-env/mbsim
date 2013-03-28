#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include <boost/filesystem.hpp>
#include <mbxmlutilstinyxml/last_write_time.h>

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {

    Integrator *integrator;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(argc>=2 && strcmp(argv[1],"--donotintegrate")==0)
      doNotIntegrate=true;
    bool stopAfterFirstStep=false;
    if(argc>=2 && strcmp(argv[1],"--stopafterfirststep")==0)
      stopAfterFirstStep=true;
  
    if(MBSimXML::preInitDynamicSystemSolver(argc, argv, dss)!=0) return 0; 
    MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  
    MBSimXML::initIntegrator(argc, argv, integrator);

    if(doNotIntegrate==false) {
      if(stopAfterFirstStep)
        MBSimXML::plotInitialState(integrator, dss);
      else
        MBSimXML::main(integrator, dss);
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

    MBSimXML::postMain(argc, argv, integrator, dss);
  }
  catch (MBSimError error) {
    error.printExceptionMessage();
    return 1;
  }

  return 0;
}

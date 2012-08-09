#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {

    Integrator *integrator;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(argc>=2 && strcmp(argv[1],"--donotintegrate")==0)
      doNotIntegrate=true;
  
    MBSimXML::preInitDynamicSystemSolver(argc, argv, dss);
  
    MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  
    MBSimXML::initIntegrator(argc, argv, integrator);
  
    if(doNotIntegrate==false) {
      MBSimXML::main(integrator, dss);
    }
  
    MBSimXML::postMain(argc, argv, integrator, dss);
  }
  catch (MBSimError error) {
    error.printExceptionMessage();
  }

  return 0;
}

#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"

using namespace std;
using namespace MBSim;

int main(int argc, char *argv[]) {
  Integrator *integrator;
  DynamicSystemSolver *dss;

  bool doNotIntegrate=false;
  if(argc>=2 && strcmp(argv[1],"--donotintegrate")==0)
    doNotIntegrate=true;

  int ret=0;

  try {
    MBSimXML::preInitDynamicSystemSolver(argc, argv, dss);
  }
  catch (MBSimError error) {
    ret=1;
    error.printExceptionMessage();
  }
  if(ret==-1) 
    return 0; // help message was printed
  if(ret!=0) 
    return ret;

  try {
    MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  }
  catch (MBSimError error) {
    ret=1;
    error.printExceptionMessage();
  }
  if(ret!=0) 
    return ret;

  try {
    MBSimXML::initIntegrator(argc, argv, integrator);
  }
  catch (MBSimError error) {
    ret=1;
    error.printExceptionMessage();
  }
  if(ret!=0) 
    return ret;

  if(doNotIntegrate==false) {
    try {
      MBSimXML::main(integrator, dss);
    }
    catch (MBSimError error) {
      ret=1;
      error.printExceptionMessage();
    }
    if(ret!=0) 
      return ret;
  }

  try {
    MBSimXML::postMain(argc, argv, integrator, dss);
  }
  catch (MBSimError error) {
    ret=1;
    error.printExceptionMessage();
  }
  if(ret!=0) 
    return ret;

  return 0;
}

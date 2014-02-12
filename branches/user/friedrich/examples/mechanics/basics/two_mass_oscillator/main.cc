#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  /* System Definition */
  // create a Dynamic system solver
  // -> Here a class System is defined derived by the dynamic system solver that assembles the bodies and force laws
  System *sys = new System("TS");

  // initialize the system using the information of the assembly
  sys->initialize();

  /* Integration */
  // define the integration
  TimeSteppingIntegrator integrator;
  // set step size
  integrator.setStepSize(1e-4);
  // set end-time of integration
  integrator.setEndTime(4.0);
  // set "sample-rate" for time-steps when information should written to the files
  integrator.setPlotStepSize(1e-3);

  // run the integration until the end-time
  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;
}


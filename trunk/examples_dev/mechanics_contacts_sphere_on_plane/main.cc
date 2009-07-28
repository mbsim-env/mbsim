#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {
  DynamicSystemSolver *sys = new System("TS");

  sys->init();
  bool eventDriven = false;

  if(eventDriven) { // Event driven time integration
    LSODARIntegrator integrator;
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  } 
  else { // time stepping integration
    double dt = 1e-4;
    TimeSteppingIntegrator integrator;
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(1.5);
    integrator.integrate(*sys);
  }

  delete sys;

  return 0;
}


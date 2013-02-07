#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  System *sys = new System("MBS");
  //sys->setImpactSolver(RootFinding);

  sys->initialize();

  bool eventDriven = false;

  if(eventDriven) { // Event driven time integration
    LSODARIntegrator integrator;
    integrator.setInitialStepSize(1e-13);
    integrator.setPlotOnRoot(false);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(2);
    double tol = 1e-4;
    sys->setProjectionTolerance(1e-15);
    sys->setgTol(1e-6);
    sys->setgdTol(1e-6);
    sys->setLaTol(1e-6);
    sys->setgddTol(1e-8);
    sys->setlaTol(1e-8);
    integrator.integrate(*sys);
  } 
  else { // time stepping integration
    double dt = 1e-4;
    sys->setLaTol(1e-2*dt);
    sys->setgdTol(1e-8);
    TimeSteppingIntegrator integrator;
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(dt);
    integrator.setEndTime(2);
    integrator.integrate(*sys);
  }

  delete sys;

  return 0;

}


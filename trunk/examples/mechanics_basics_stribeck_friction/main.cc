#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  bool eventDriven = true;

  if(eventDriven) {
    sys->setProjectionTolerance(1e-15);
    sys->setgTol(1e-6);
    sys->setgdTol(1e-6);
    sys->setLaTol(1e-6);
    sys->setlaTol(1e-8);
    sys->setgddTol(1e-8);
    sys->initialize();

    LSODARIntegrator integrator;
    integrator.setPlotOnRoot(false);
    integrator.setInitialStepSize(1e-8);

    integrator.setEndTime(0.5);
    integrator.setPlotStepSize(1e-3);

    integrator.integrate(*sys);
  } 
  else {
    sys->setImpactSolver(RootFinding);
    sys->setLinAlg(PseudoInverse);
    sys->setNumJacProj(true);
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    TimeSteppingIntegrator integrator;

    integrator.setEndTime(0.5);
    integrator.setStepSize(1e-4);
    integrator.setPlotStepSize(5e-4);

    integrator.integrate(*sys);
  }

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}


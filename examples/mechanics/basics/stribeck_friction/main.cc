#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  bool eventDriven = true;

  if(eventDriven) {
    sys->setProjectionTolerance(1e-15);
    sys->setGeneralizedRelativePositionTolerance(1e-6);
    sys->setGeneralizedRelativeVelocityTolerance(1e-6);
    sys->setGeneralizedImpulseTolerance(1e-6);
    sys->setGeneralizedForceTolerance(1e-8);
    sys->setGeneralizedRelativeAccelerationTolerance(1e-8);
    sys->initialize();

    LSODEIntegrator integrator;
    integrator.setInitialStepSize(1e-8);

    integrator.setEndTime(0.5);
    integrator.setPlotStepSize(1e-3);

    integrator.integrate(*sys);
  } 
  else {
    sys->setImpactSolver(DynamicSystemSolver::rootfinding);
    sys->setLinearAlgebra(DynamicSystemSolver::pseudoinverse);
    sys->setStopIfNoConvergence(true,true);
    sys->initialize();

    TimeSteppingIntegrator integrator;

    integrator.setEndTime(0.5);
    integrator.setStepSize(1e-4);
    integrator.setPlotStepSize(5e-4);

    integrator.integrate(*sys);
  }

  cout << "finished"<<endl;

  delete sys;

  return 0;
}


#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[])
{
  System *sys = new System("MBS");
  //sys->setImpactSolver(RootFinding);

  sys->initialize();

  bool eventDriven = false;

  if(eventDriven) { // Event driven time integration
    LSODEIntegrator integrator;
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(2);
    double tol = 1e-4;
    sys->setProjectionTolerance(1e-15);
    sys->setGeneralizedRelativePositionTolerance(1e-6);
    sys->setGeneralizedRelativeVelocityTolerance(1e-6);
    sys->setGeneralizedImpulseTolerance(1e-6);
    sys->setGeneralizedRelativeAccelerationTolerance(1e-8);
    sys->setGeneralizedForceTolerance(1e-8);
    integrator.integrate(*sys);
  } 
  else { // time stepping integration
    double dt = 1e-4;
    sys->setGeneralizedImpulseTolerance(1e-2*dt);
    sys->setGeneralizedRelativeVelocityTolerance(1e-8);
    TimeSteppingIntegrator integrator;
    integrator.setStepSize(dt);
    integrator.setPlotStepSize(1e-2);
    integrator.setEndTime(2);
    integrator.integrate(*sys);
  }

  delete sys;

  return 0;

}


#include <mbsim/integrators/integrators.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/environment.h>
#include "system.h"

#include <time.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  clock_t start, end;

  DynamicSystemSolver * sys = new System("MBS");
  MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0;-9.81;0]");
  sys->initialize();
  sys->setStopIfNoConvergence(true,true);

  Integrator* integrator;
  bool eventDriven = true;
  if(eventDriven) { // Event driven time integration
    sys->setProjectionTolerance(1e-15);
    sys->setGeneralizedRelativePositionTolerance(1e-6);
    sys->setGeneralizedRelativeVelocityTolerance(1e-6);
    sys->setGeneralizedImpulseTolerance(1e-6);
    sys->setGeneralizedRelativeAccelerationTolerance(1e-8);
    sys->setGeneralizedForceTolerance(1e-8);
    integrator = new LSODEIntegrator;
    static_cast<LSODEIntegrator*>(integrator)->setMaximumStepSize(1e-2);
    static_cast<LSODEIntegrator*>(integrator)->setInitialStepSize(1e-10);
    static_cast<LSODEIntegrator*>(integrator)->setAbsoluteTolerance(1e-7);
    static_cast<LSODEIntegrator*>(integrator)->setRelativeTolerance(1e-7);
    static_cast<LSODEIntegrator*>(integrator)->setToleranceForPositionConstraints(1e-5);
    static_cast<LSODEIntegrator*>(integrator)->setToleranceForVelocityConstraints(1e-5);
  }
  else { // time stepping integration
    double dt = 1e-4;
    sys->setGeneralizedImpulseTolerance(1e-2*dt);
    sys->setGeneralizedRelativeVelocityTolerance(1e-8);
    integrator = new TimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt);
  }
  //TimeSteppingIntegrator integrator; integrator.setStepSize(1e-4);
  //DOPRI5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  //LSODEIntegrator integrator; integrator.setMaximalStepSize(1e-4);
  //RADAU5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  integrator->setEndTime(2e-0);
  integrator->setPlotStepSize(1e-3);
  start=clock();
  integrator->integrate(*sys);
  end=clock();
  cout << "Integration time: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;

  delete sys;
  delete integrator;

  return 0;
}



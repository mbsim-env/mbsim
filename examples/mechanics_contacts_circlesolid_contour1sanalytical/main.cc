#include <mbsim/integrators/integrators.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/environment.h>
#include "system.h"

#include <time.h>

using namespace std;
using namespace MBSim;
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
    sys->setgTol(1e-6);
    sys->setgdTol(1e-6);
    sys->setLaTol(1e-6);
    sys->setgddTol(1e-8);
    sys->setlaTol(1e-8);
    integrator = new LSODARIntegrator;
    static_cast<LSODARIntegrator*>(integrator)->setPlotOnRoot(false);
    static_cast<LSODARIntegrator*>(integrator)->setMaximalStepSize(1e-3);
    static_cast<LSODARIntegrator*>(integrator)->setInitialStepSize(1e-10);
  } 
  else { // time stepping integration
    double dt = 1e-4;
    sys->setLaTol(1e-2*dt);
    sys->setgdTol(1e-8);
    integrator = new TimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt);
  }
  //TimeSteppingIntegrator integrator; integrator.setStepSize(1e-4);
  //DOPRI5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  //LSODEIntegrator integrator; integrator.setMaximalStepSize(1e-4);
  //RADAU5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  integrator->setEndTime(2e-0);
  integrator->setPlotStepSize(1e-3);
  integrator->setOutput(true);
  start=clock();
  integrator->integrate(*sys);
  end=clock();
  cout << "Integration Time TimeStepping: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;

  sys->closePlot();
  delete sys;

  return 0;
}



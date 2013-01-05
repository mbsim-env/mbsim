#include <mbsim/integrators/integrators.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/environment.h>
#include "system.h"

#include "mbsim/utils/stopwatch.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  DynamicSystemSolver * sys = new System("MBS");
  MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0;-9.81;0]");
  sys->initialize();

  StopWatch Watch;

  TimeSteppingIntegrator integrator; integrator.setStepSize(1e-4);
  integrator.setEndTime(2e-0);
  integrator.setPlotStepSize(1e-4);
  integrator.setOutput(true);
  Watch.start();
  integrator.integrate(*sys);
  cout << "Integration Time TimeStepping: " << Watch.stop() << " [s]." << endl;

  sys->closePlot();
  delete sys;

  return 0;
}



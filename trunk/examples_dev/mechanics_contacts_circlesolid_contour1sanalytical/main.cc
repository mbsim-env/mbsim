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
  sys->init();

//  TimeSteppingIntegrator integrator; integrator.setStepSize(1e-4);
//  DOPRI5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  LSODEIntegrator integrator; integrator.setMaximalStepSize(1e-4);
  //RADAU5Integrator integrator; integrator.setMaximalStepSize(1e-4);
  integrator.setEndTime(2e-0);
  integrator.setPlotStepSize(1e-4);
  integrator.setOutput(true);
  start=clock();
  integrator.integrate(*sys);
  end=clock();
  cout << "Integration Time TimeStepping: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;

  sys->closePlot();
  delete sys;

  return 0;
}



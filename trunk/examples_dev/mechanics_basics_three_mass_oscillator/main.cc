#include "system.h"

#include "mbsim/integrators/integrators.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/environment.h"

#include <time.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main(int argc, char *argv[]) {
  bool reorganize = true;

  DynamicSystemSolver *sys = new DynamicSystemSolver("MBS");
  sys->addGroup(new System(0,reorganize));
  sys->addGroup(new System(1,reorganize));
  sys->addGroup(new System(2,reorganize));
  MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0;0;0]");

  sys->setConstraintSolver(LinearEquations);
  sys->setImpactSolver(LinearEquations);
  sys->setgdTol(1e-9);
  sys->setReorganizeHierarchy(reorganize);

  sys->init();

  double dtPlot=1e-4;
  clock_t start, end;
  RADAU5Integrator integrator;
  integrator.setEndTime(5e-1);
  integrator.setPlotStepSize(dtPlot);
  integrator.setOutput(true);
  integrator.setMaximalStepSize(dtPlot);
  start=clock();
  integrator.integrate(*sys);
  end=clock();
  cout << "Integration Time: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;

  sys->closePlot();
  delete sys;

  return 0;
}


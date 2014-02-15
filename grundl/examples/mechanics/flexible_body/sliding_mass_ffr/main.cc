#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {

  StopWatch sw;

  sw.start();
  SlidingMass *sys = new SlidingMass("MBS");

  sys->setImpactSolver(RootFinding);
  sys->setStopIfNoConvergence(true, true);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(5e-1);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(1e-3);
  integrator.integrate(*sys);

  sys->closePlot();

  cout << "finished in " << sw.stop() << endl;

  delete sys;

  return 0;

}


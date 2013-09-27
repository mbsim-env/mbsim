#include "system.h"
#include <mbsim/integrators/integrators.h>
#include "mbsim/utils/stopwatch.h"

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {

  System *sys = new System("MBS");

  StopWatch Timer;

  sys->setStopIfNoConvergence(true,true);
  sys->setMaxIter(100000); // set up to 100000 because of "No Convergence" in only ONE step
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(5e-4);
  integrator.setStepSize(1e-6);
  integrator.setPlotStepSize(1e-4);

  Timer.start();
  integrator.integrate(*sys);

  cout << "CPU-Time = " << Timer.stop() << endl;

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;
}


#include "system.h"
#include <mbsim/integrators/integrators.h>
#include "mbsim/utils/stopwatch.h"

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {

  System *sys = new System("MBS");

  StopWatch Timer;

  sys->setStopIfNoConvergence(true,true);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(0.05);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(1e-4);

  Timer.start();
  integrator.integrate(*sys);

  cout << "CPU-Time = " << Timer.stop() << endl;

  cout << "finished"<<endl;

  delete sys;

  return 0;
}


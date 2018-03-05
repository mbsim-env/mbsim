#include "system.h"

#include <mbsim/integrators/integrators.h>
#include "mbsim/utils/stopwatch.h"

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main(int argc, char* argv[]) {

  ALETester sys("MBS");

  StopWatch Timer;

  sys.setStopIfNoConvergence(true,true);
  sys.setMaximumIterations(100000); // set up to 100000 because of "No Convergence" in only ONE step
  sys.initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(5e-3);
  integrator.setStepSize(1e-6);
  integrator.setPlotStepSize(1e-4);

  Timer.start();
  integrator.integrate(sys);

  return 0;
}


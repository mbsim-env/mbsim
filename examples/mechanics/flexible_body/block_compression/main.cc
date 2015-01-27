#include "block_compression.h"
#include <mbsim/integrators/integrators.h>
#include "mbsim/utils/stopwatch.h"

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main(int argc, char* argv[]) {

  BlockCompression *sys = new BlockCompression("BlockCompressor");

  StopWatch Timer;

  sys->setStopIfNoConvergence(true, true);
  sys->setMaxIter(100000); // set up to 100000 because of "No Convergence" in only ONE step
  sys->initialize();
  sys->setFlushEvery(1);

  Integrator* integrator;

  if (0) {
    integrator = new TimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(1e-6);
  }
  else {
    integrator = new QuasiStaticIntegrator;
    static_cast<QuasiStaticIntegrator*>(integrator)->setStepSize(1e-7);
    static_cast<QuasiStaticIntegrator*>(integrator)->setTolerance(1e-1);
  }

  integrator->setPlotStepSize(1e-7);
  integrator->setEndTime(5e-4);
  Timer.start();
  integrator->integrate(*sys);

  cout << "CPU-Time = " << Timer.stop() << endl;

  cout << "finished" << endl;

  delete integrator;
  delete sys;

  return 0;
}


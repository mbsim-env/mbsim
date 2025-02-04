#include "block_compression.h"
#include <mbsim/integrators/integrators.h>
#include "mbsim/utils/stopwatch.h"

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {

  BlockCompression *sys = new BlockCompression("BlockCompressor");

  StopWatch Timer;

  sys->setStopIfNoConvergence(true, true);
  sys->setMaximumNumberOfIterations(100000); // set up to 100000 because of "No Convergence" in only ONE step
  sys->initialize();

  Integrator* integrator;

  if (1) {
    integrator = new RADAU5Integrator;
    dynamic_cast<RADAU5Integrator*>(integrator)->setFormalism(RADAU5Integrator::DAE2);
  }
  else {
    integrator = new QuasiStaticIntegrator;
    static_cast<QuasiStaticIntegrator*>(integrator)->setStepSize(1e-6);
    static_cast<QuasiStaticIntegrator*>(integrator)->setgTolerance(1e-1);
    static_cast<QuasiStaticIntegrator*>(integrator)->sethTolerance(1e-1);
    static_cast<QuasiStaticIntegrator*>(integrator)->setmaxExtraPolate(1);
    static_cast<QuasiStaticIntegrator*>(integrator)->setupdateJacobianEvery(1);
  }

  integrator->setPlotStepSize(1e-6);
  integrator->setEndTime(2.1e-5);
  Timer.start();
  integrator->integrate(*sys);

  cout << "CPU-Time = " << Timer.stop() << endl;

  cout << "finished" << endl;

  delete integrator;
  delete sys;

  return 0;
}


#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;

int main (int argc, char* argv[]) {

  bool setValued=false;

  DynamicSystemSolver *sys = new System("MBS", setValued);
  sys->setPlotFeature(plotRecursive, enabled);
  sys->setPlotFeature(state, enabled);
  sys->setPlotFeature(globalPosition, enabled);
  sys->setStopIfNoConvergence(true, true);
  sys->init();

  Integrator * integrator;
  if (setValued) {
    integrator = new TimeSteppingIntegrator();
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(1e-4);
  }
  else
    integrator = new LSODEIntegrator();
  integrator->setEndTime(4e-0);
  integrator->setPlotStepSize(1e-3);
  integrator->integrate(*sys);

  sys->closePlot();
  delete sys;
  delete integrator;

  return 0;
}

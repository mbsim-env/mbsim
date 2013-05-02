#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
  sys.setConstraintSolver(GaussSeidel);
  sys.setImpactSolver(GaussSeidel);
  sys.setFlushEvery(100);
  sys.initialize();

  Integrator* integrator;
  //integrator.setDriftCompensation(true);

  sys.setStopIfNoConvergence(true, true);
  bool eventDriven = true;
  if(eventDriven) { // Event driven time integration
    sys.setProjectionTolerance(1e-15);
    sys.setgTol(1e-6);
    sys.setgdTol(1e-8);
    sys.setLaTol(1e-8);
    sys.setgddTol(1e-10);
    sys.setlaTol(1e-10);
    integrator = new LSODARIntegrator;
    static_cast<LSODARIntegrator*>(integrator)->setPlotOnRoot(false);
    static_cast<LSODARIntegrator*>(integrator)->setInitialStepSize(1e-10);
    static_cast<LSODARIntegrator*>(integrator)->setMaximalStepSize(1e-2);
  } 
  else { // time stepping integration
    double dt = 1e-5;
    sys.setLaTol(1e-2*dt);
    sys.setgdTol(1e-8);
    integrator = new TimeSteppingIntegrator;
    //integrator = new ThetaTimeSteppingIntegrator;
    static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt);
    //static_cast<ThetaTimeSteppingIntegrator*>(integrator)->setTheta(0.75);
  }

  integrator->setEndTime(5);
  integrator->setPlotStepSize(1.e-3);
  integrator->integrate(sys);

  sys.closePlot();

  return 0;
}


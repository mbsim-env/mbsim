#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[]) {

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->dropContactMatrices();
  sys->setConstraintSolver(FixedPointSingle);
  sys->setImpactSolver(FixedPointSingle);
  sys->setStrategy(local);
  sys->initialize();

  Integrator *integrator;

  const int selIntegrator = 0;
  switch(selIntegrator) {
    default:{
              integrator = new TimeSteppingIntegrator();
              static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(1.0e-5);
            }
            break;
    case 1: {
              integrator = new ThetaTimeSteppingIntegrator();
              static_cast<ThetaTimeSteppingIntegrator*>(integrator)->setStepSize(5.0e-4);
            }
            break;
            //LSODARIntegrator integrator;
  }

  //integrator.setTheta(0.5);
  integrator->setEndTime(5.0e-2);
  integrator->setPlotStepSize(5e-4);
  integrator->integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete integrator;
  delete sys;

  return 0;

}


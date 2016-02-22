#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main (int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setStopIfNoConvergence(false);
//  sys->dropContactMatrices();
  sys->setConstraintSolver(DynamicSystemSolver::FixedPointSingle);
  sys->setImpactSolver(DynamicSystemSolver::FixedPointSingle);
  sys->setStrategy(DynamicSystemSolver::local);
  sys->initialize();

  sys->setgdTol (1.0e-12);
  sys->setgddTol(1.0e-11);
  sys->setlaTol (1.0e-9);
  sys->setLaTol (1.0e-9);
  sys->setMaxIter(pow(150*sys->getlaSize(),0.85));

  Integrator *integrator;

  const int selIntegrator = 0;
  double dt_const = 0.0;
  switch(selIntegrator) {
    default:{
              integrator = new TimeSteppingIntegrator();
              dt_const = 1.0e-6;
              static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt_const);
            }
            break;
    case 1: {
              integrator = new ThetaTimeSteppingIntegrator();
              dt_const = 2.0e-5;
              static_cast<ThetaTimeSteppingIntegrator*>(integrator)->setStepSize(dt_const);
              static_cast<ThetaTimeSteppingIntegrator*>(integrator)->setTheta(1.0);
            }
            break;
            //LSODARIntegrator integrator;
    case 2: {
              integrator = new LSODARIntegrator();
              //static_cast<LSODARIntegrator*>(integrator)->setStepSize(5.0e-4);
            }
            break;
  }

  integrator->setEndTime(1.6e-1);
  integrator->setEndTime(4.e-4);
  integrator->setPlotStepSize(max(1e-4,dt_const));
  integrator->integrate(*sys);

  sys->closePlot();

  cout << "finished"<<endl;

  delete integrator;
  delete sys;

  return 0;

}


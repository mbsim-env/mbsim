#include "system.h"
#include <mbsim/integrators/integrators.h>
#include <mbsim/utils/stopwatch.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  StopWatch sw;
  sw.start();

  System *sys = new System("TS", 0, 12);

//  sys->setImpactSolver(RootFinding);
//  sys->setConstraintSolver(RootFinding);
//  sys->setLinAlg(PseudoInverse);
  sys->setNumJacProj(true);
  sys->initialize();

  TimeSteppingIntegrator integrator;
  integrator.setStepSize(1e-4);

  integrator.setEndTime(.4);
  integrator.setPlotStepSize(1e-4);

  integrator.integrate(*sys);



  cout << "*********************" << endl;
  cout << " Time = " << sw.stop() << " s " << endl;
  cout << "*********************" << endl;
  cout << "finished"<<endl;
  delete sys;

  return 0;
}


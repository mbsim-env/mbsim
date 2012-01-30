#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
//  sys.setImpactSolver(RootFinding);
  double dt = 1e-4;
  sys.setLaTol(1e-2*dt);
  sys.setgdTol(1e-8);
  sys.initialize();

  //ThetaTimeSteppingIntegrator integrator;
  TimeSteppingIntegrator integrator;
  integrator.setStepSize(dt);
  //integrator.setDriftCompensation(true);
  //integrator.setTheta(0.75);
  integrator.setEndTime(5);
  integrator.setPlotStepSize(5.e-3);
  
  integrator.integrate(sys);

  sys.closePlot();

  return 0;
}


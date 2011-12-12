#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
//  sys.setImpactSolver(RootFinding);
  sys.initialize();

  ThetaTimeSteppingIntegrator integrator;
  integrator.setStepSize(1.e-4);
  //integrator.setDriftCompensation(true);
  integrator.setTheta(0.75);
  integrator.setEndTime(5);
  integrator.setPlotStepSize(5.e-3);
  
  integrator.integrate(sys);

  sys.closePlot();

  return 0;
}


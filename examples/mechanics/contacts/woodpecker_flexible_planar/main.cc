#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
//  sys.setImpactSolver(RootFinding);
  sys.initialize();
  sys.setGeneralizedImpulseTolerance(1e-5);
  sys.setGeneralizedRelativeVelocityTolerance(1e-5);

//  TimeSteppingSSCIntegrator integrator;
  TimeSteppingIntegrator integrator;
  integrator.setEndTime(0.2);
  integrator.setStepSize(1e-5);
  integrator.setPlotStepSize(5.e-3);
  
  integrator.integrate(sys);

  return 0;
}


#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
//  sys.setImpactSolver(RootFinding);
  sys.initialize();

  TimeSteppingSSCIntegrator integrator;
  //integrator.setStepSize(1.e-5);
  //integrator.setDriftCompensation(true);
  integrator.setEndTime(5e-2);
  integrator.setPlotStepSize(5.e-3);
  
  integrator.integrate(sys);

  sys.closePlot();

  return 0;
}


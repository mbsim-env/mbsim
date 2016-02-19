#include "woodpecker.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
//  sys.setImpactSolver(RootFinding);
  sys.initialize();

  TimeSteppingSSCIntegrator integrator;
  integrator.setEndTime(4e-2);
  integrator.setPlotStepSize(5.e-3);
  
  integrator.integrate(sys);

  sys.closePlot();

  return 0;
}


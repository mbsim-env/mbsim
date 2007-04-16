#include "woodpecker.h"
#include "integrators.h"

using namespace std;

using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("WP");
  sys.setSolver(RootFinding);
//  sys.setSolver(solveGS);
//  sys.setSolver(projectGS);
  sys.init();

  TimeSteppingIntegrator integrator;
  integrator.setdt(1.e-5);
  integrator.settEnd(5.0);
  integrator.setdtPlot(1.e-3);
  
// integrator.setWarnLevel(0); defautl: 0
// integrator.setOutput(false); default: true

  integrator.integrate(sys);

  return 0;
}


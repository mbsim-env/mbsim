#include "woodpecker.h"
#include "integrators.h"

using namespace std;

using namespace MBSim;

int main (int argc, char* argv[]) {

  Woodpecker sys("MBS");
  sys.setSolver(RootFinding);
  sys.init();

  TimeSteppingIntegrator integrator;
  integrator.setdt(1.e-5);
  integrator.settEnd(5);
  integrator.setdtPlot(1.e-2);
  
  integrator.integrate(sys);

  return 0;
}


#include "rocking_rod.h"
#include <integrators.h>

using namespace std;

int main (int argc, char* argv[]) {

  RockingRod sys("MBS");
  sys.init();

  TimeSteppingIntegrator integrator;
  integrator.setdt(1e-3);
  integrator.settEnd(3);
  integrator.setdtPlot(1e-2);

  integrator.integrate(sys);
  cout << "finished"<<endl;

  return 0;

}


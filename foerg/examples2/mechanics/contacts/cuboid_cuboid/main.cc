#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

int main (int argc, char* argv[]) {

  double dt = 1e-4;

  System sys("MBS");
  sys.setLaTol(1e-1*dt);
  sys.setgdTol(1e-4);
  sys.setMaxIter(100000);
  sys.setrMax(1.0);
  sys.initialize();


  TimeSteppingIntegrator integrator;
  integrator.setStepSize(dt);

  integrator.setEndTime(0.30);
  integrator.setPlotStepSize(1e-3);

  integrator.integrate(sys);

  cout << "finished"<<endl;

  return 0;

}


#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

int main (int argc, char* argv[]) {

  double dt = 1e-4;

  System sys("MBS");
  sys.setGeneralizedImpulseTolerance(1e-1*dt);
  sys.setGeneralizedRelativeVelocityTolerance(1e-4);
  sys.initialize();

  TimeSteppingIntegrator integrator;
  integrator.setStepSize(dt);

  integrator.setEndTime(0.7);
  integrator.setPlotStepSize(1e-2);

  integrator.integrate(sys);

  cout << "finished"<<endl;

  return 0;

}


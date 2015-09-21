#include <mbsim/integrators/integrators.h>
#include "flexible_rotor_ehd.h"

using namespace MBSim;
using namespace MBSimIntegrator;
using namespace std;

int main (int argc, char* argv[]) {

  FlexibleRotorEHD *sys = new FlexibleRotorEHD("MBS");
  sys->setFlushEvery(1);
  sys->initialize();

  TimeSteppingIntegrator integrator;
  integrator.setEndTime(1e-5);
  integrator.setStepSize(1e-6);
  integrator.setPlotStepSize(1e-6);

  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}


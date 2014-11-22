#include <mbsim/integrators/integrators.h>
#include "../mass_spring_rotating_quasi/system.h"

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

int main(int argc, char* argv[]) {
  // build single modules
  {
    System *sys = new System("QuasiStatic");

    // add modules to overall dynamical system
    sys->initialize();

    QuasiStaticIntegrator integrator;

    integrator.setStepSize(1e-1);
    integrator.setEndTime(10.0);
    integrator.setPlotStepSize(1e-1);

    integrator.integrate(*sys);
    delete sys;
  }

  {
    System *sys = new System("TimeStepping");

    // add modules to overall dynamical system
    sys->initialize();

    TimeSteppingIntegrator integrator;

    integrator.setStepSize(1e-3);
    integrator.setEndTime(10.0);
    integrator.setPlotStepSize(1e-1);

    integrator.integrate(*sys);
    delete sys;
  }

  cout << "finished" << endl;

  return 0;
}


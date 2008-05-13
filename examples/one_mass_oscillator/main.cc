#include "one_mass_oscillator.h"
#include <integrators.h>
#include <sstream>

using namespace std;

int main (int argc, char* argv[]) {

  OneMassOscillator sys("oscillator");
  sys.setProjectDirectory("plot");
  sys.setPlotLevel(3);
  sys.init();

  LSODEIntegrator integrator;
  integrator.settEnd(5);
  integrator.setaTol(1.e-7);
  integrator.setdtPlot(2.5e-4);

  integrator.integrate(sys);

  return 0;
}


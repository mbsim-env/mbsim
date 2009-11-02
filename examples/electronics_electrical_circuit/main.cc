#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new System("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->initialize();
  
  // LSODEIntegrator integrator;
  // RKSuiteIntegrator integrator;
  // RADAU5Integrator integrator;
  // TimeSteppingIntegrator integrator;
  // integrator.setdt(1e-4);
  //
  DOPRI5Integrator integrator;

  integrator.setEndTime(2.0);
  integrator.setPlotStepSize(1e-2);

  integrator.integrate(*sys);

  sys->closePlot();

  delete sys;

  return 0;

}


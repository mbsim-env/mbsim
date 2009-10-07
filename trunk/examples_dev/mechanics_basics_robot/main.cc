#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new Robot("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->initialize();

  //LSODARIntegrator integrator;
  //LSODEIntegrator integrator;
  // RKSuiteIntegrator integrator;
  //RADAU5Integrator integrator;
  DOPRI5Integrator integrator;
  // TimeSteppingIntegrator integrator;

  integrator.setEndTime(14.0);
  integrator.setPlotStepSize(1e-2);
  // integrator.setStepSize(1e-5);
  //integrator.setRelativeTolerance(1e-8);
  //integrator.setAbsoluteTolerance(1e-8);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;

}


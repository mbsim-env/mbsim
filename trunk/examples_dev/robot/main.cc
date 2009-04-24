#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace MBSim;
using namespace std;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new Robot("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->setProjectDirectory("plot");
  sys->init();

  //LSODARIntegrator integrator;
  //LSODEIntegrator integrator;
  // RKSuiteIntegrator integrator;
  //RADAU5Integrator integrator;
  DOPRI5Integrator integrator;
  // TimeSteppingIntegrator integrator;

  integrator.settEnd(14.0);
  integrator.setdtPlot(1e-2);
  // integrator.setdt(1e-5);
  //integrator.setrTol(1e-8);
  //integrator.setaTol(1e-8);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;

}


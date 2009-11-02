#include "pendulum.h"
#include <integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  MultiBodySystem *sys = new Pendulum("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->init();
  
  // LSODEIntegrator integrator;
  // RKSuiteIntegrator integrator;
  // RADAU5Integrator integrator;
  // TimeSteppingIntegrator integrator;
  // integrator.setdt(1e-4);
  //
  DOPRI5Integrator integrator;

  integrator.settEnd(10.0);
  integrator.setdtPlot(1e-3);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;

}


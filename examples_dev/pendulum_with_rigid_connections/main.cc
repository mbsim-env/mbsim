#include "system.h"
#include <integrators.h>

using namespace std;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  MultiBodySystem *sys = new System("MBS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->setProjectDirectory("plot");
  //sys->setSolver(GaussSeidel);
  //sys->setSolver(RootFinding);
  sys->init();
  
  //LSODARIntegrator integrator;
  //LSODEIntegrator integrator;
 // RKSuiteIntegrator integrator;
//  RADAU5Integrator integrator;
  //TimeSteppingIntegrator integrator;
  //integrator.setdt(1e-4);

  DOPRI5Integrator integrator;

  integrator.settEnd(10.1);
  integrator.setdtPlot(1e-2);

  integrator.integrate(*sys);

  cout << "finished"<<endl;

  delete sys;

  return 0;

}


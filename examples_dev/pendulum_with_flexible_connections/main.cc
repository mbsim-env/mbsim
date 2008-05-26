#include "system.h"
#include <integrators.h>

using namespace std;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  MultiBodySystem *sys = new System("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->setProjectDirectory("plot");
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
 // integrator.setdt(1e-5);
  //integrator.setrTol(1e-8);
  //integrator.setaTol(1e-8);

  double s1 = clock(); 

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;

}


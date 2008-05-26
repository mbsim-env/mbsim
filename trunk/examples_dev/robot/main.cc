#include "robot.h"
#include <integrators.h>

using namespace std;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  MultiBodySystem *sys = new Robot("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->setProjectDirectory("plot");
  sys->init();
  
  //LSODARIntegrator integrator;
  //LSODEIntegrator integrator;
 // RKSuiteIntegrator integrator;
  //RADAU5Integrator integrator;
  DOPRI5Integrator integrator;
// TimeSteppingIntegrator integrator;

  integrator.settEnd(2.0);
  integrator.setdtPlot(1e-3);
 // integrator.setdt(1e-5);
  //integrator.setrTol(1e-8);
  //integrator.setaTol(1e-8);

  integrator.integrate(*sys);
  cout << "finished"<<endl;
  delete sys;

  return 0;

}


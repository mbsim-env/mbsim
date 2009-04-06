#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new System("TS");

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->init();
  
  // LSODEIntegrator integrator;
  // RKSuiteIntegrator integrator;
  // RADAU5Integrator integrator;
  // TimeSteppingIntegrator integrator;
  // integrator.setdt(1e-4);
  //
  DOPRI5Integrator integrator;

  integrator.settEnd(2.0);
  integrator.setdtPlot(1e-2);

//  int zSize=sys->getzSize();
//  double t = 0.0;
//  fmatvec::Vec z(zSize);
//  z(2) = 1;
//  z(1) = 0.4;
//  sys->initz(z);          
//  sys->zdot(z, t);
//  cout << sys->getM() << endl;
//  cout << sys->geth() << endl;
//
 
  integrator.integrate(*sys);

  sys->closePlot();

  delete sys;

  return 0;

}


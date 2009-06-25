#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new Pendulum("TS");
  sys->setReorganizeHierarchy(true);

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->init();
  
  DOPRI5Integrator integrator;

  integrator.settEnd(2.0);
  integrator.setdtPlot(1e-2);

  integrator.integrate(*sys);

  cout << "finished"<<endl;
  delete sys;

  return 0;

}


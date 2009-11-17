#include "system.h"
#include <mbsim/integrators/integrators.h>

using namespace std;
using namespace MBSim;

bool setValued = true;

int main (int argc, char* argv[])
{
  // Einzelne Bausteine des MKS erschaffen
  DynamicSystemSolver *sys = new System("TS");
  sys->setConstraintSolver(GaussSeidel);
  sys->setImpactSolver(GaussSeidel);

  // Bausteine zum Gesamtsystem zusammenfuegen (zu einem DGL-System) 
  sys->initialize();
  
   Integrator *integrator;
   if(setValued) {
     integrator = new TimeSteppingIntegrator;
     double dt = 1e-6;
     static_cast<TimeSteppingIntegrator*>(integrator)->setStepSize(dt);
     sys->setLaTol(1e-2*dt);
     sys->setgdTol(1e-8);
   }
   else {

   integrator = new RADAU5Integrator;
   static_cast<RADAU5Integrator*>(integrator)->setMaximalStepSize(1e-5);
   static_cast<RADAU5Integrator*>(integrator)->setRelativeTolerance(1e-10);
   }

  //DOPRI5Integrator integrator;
   //LSODEIntegrator integrator;

  integrator->setPlotStepSize(1e-5);
  integrator->setEndTime(1e-2);
  integrator->integrate(*sys);

  sys->closePlot();

  delete sys;

  return 0;

}


#include "system.h"

#include "mbsim/integrators/integrators.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/environment.h"

#include <time.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace fmatvec;

int main(int argc, char *argv[]) {
  DynamicSystemSolver *sys = new DynamicSystemSolver("MBS");
  double dDisk=0.05;
  sys->addFrame(new FixedRelativeFrame("Q1",1.5*dDisk*Vec("[1;0;0]")+1.5*dDisk*Vec("[0;0;-1]")));
  sys->addFrame(new FixedRelativeFrame("Q2",3.*dDisk*Vec("[1;0;0]")+1.5*dDisk*Vec("[0;0;-1]")));
  sys->addFrame(new FixedRelativeFrame("Q0",1.5*dDisk*Vec("[0;0;-1]")));
  System *group = new System(0,dDisk);
  sys->addGroup(group);
  group->setFrameOfReference(sys->getFrame("Q0"));
  group = new System(1,dDisk);
  sys->addGroup(group);
  group->setFrameOfReference(sys->getFrame("Q1"));
  group = new System(2,dDisk);
  sys->addGroup(group);
  group->setFrameOfReference(sys->getFrame("Q2"));
  MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0;0;0]");

  sys->setConstraintSolver(DynamicSystemSolver::LinearEquations);
  sys->setImpactSolver(DynamicSystemSolver::LinearEquations);
  sys->setGeneralizedRelativeVelocityTolerance(1e-9);

  sys->initialize();

  double dtPlot=1e-4;
  clock_t start, end;
  RADAU5Integrator integrator;
  integrator.setEndTime(5e-1);
  integrator.setPlotStepSize(dtPlot);
  integrator.setMaximumStepSize(dtPlot);
  start=clock();
  integrator.integrate(*sys);
  end=clock();
  cout << "Integration Time: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;

  delete sys;

  return 0;
}


#include <mbsim/dynamic_system_solver.h>
#include <mbsim/rigid_body.h>
#include <mbsim/integrators/integrators.h>
#include "line.h"
#include "press_bound.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;

int main (int argc, char* argv[])
{
  DynamicSystemSolver *sys=new DynamicSystemSolver("RelHyd");

/*

     O"p1"       O"p3"
      \          |
       \"L1"     |"L3"
        \        |
          ------   ------ O"p0"
        /  "Ld"     "LD"
       /"L2
      /
     O"p2"

*/

  Line *line1=new Line("L1");
  sys->addObject(line1);

  PressBound *bound1=new PressBound("p1");
  bound1->setPressure(2);
  bound1->addInConnection(line1);
  sys->addLink(bound1);

  Line *line2=new Line("L2");
  sys->addObject(line2);

  PressBound *bound2=new PressBound("p2");
  bound2->setPressure(2);
  bound2->addInConnection(line2);
  sys->addLink(bound2);

  Line *lined=new Line("Ld");
  lined->addDependency(line1);
  lined->addDependency(line2);
  sys->addObject(lined);

  Line *line3=new Line("L3");
  sys->addObject(line3);

  PressBound *bound3=new PressBound("p3");
  bound3->setPressure(2);
  bound3->addInConnection(line3);
  sys->addLink(bound3);

  Line *lineD=new Line("LD");
  lineD->addDependency(lined);
  lineD->addDependency(line3);
  sys->addObject(lineD);

  PressBound *bound0=new PressBound("p0");
  bound0->setPressure(1);
  bound0->addOutConnection(lineD);
  sys->addLink(bound0);




  //RigidBody *body1=new RigidBody("Body1");
  //body1->setMass(1);
  //body1->setFrameOfReference(sys->getFrame("I"));
  //body1->setInertiaTensor(SymMat(3,EYE));
  //body1->setTranslation(new LinearTranslation("[1;0;0]"));
  //sys->addObject(body1);

  //RigidBody *body2=new RigidBody("Body2");
  //body2->setMass(1);
  //body2->setFrameOfReference(body1->getFrame("C"));
  //body2->setInertiaTensor(SymMat(3,EYE));
  //body2->setTranslation(new LinearTranslation("[1;0;0]"));
  //sys->addObject(body2);



  sys->initialize();

  DOPRI5Integrator integrator;

  integrator.setEndTime(4.0);
  integrator.setPlotStepSize(1e-3);

  integrator.integrate(*sys);
  sys->closePlot();

  delete sys;

  return 0;

}


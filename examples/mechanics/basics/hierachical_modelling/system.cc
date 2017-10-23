#include "system.h"
#include "group2.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/environment.h"
#include "mbsim/objects/object.h"
#include "mbsim/links/link.h"
#include "mbsim/links/spring_damper.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  
  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81*0;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);

  Group2 *group1 = new Group2("Hauptgruppe1");
  addGroup(group1);

  Group2 *group2 = new Group2("Hauptgruppe2");
  group2->setPlotFeatureRecursive(derivativeOfGeneralizedPosition, enabled);
  group2->setPlotFeatureRecursive(generalizedAcceleration, enabled);
  group2->setPlotFeature(separateFilePerGroup, enabled);
  Vec r(3);
  r(0) = 2;
  SqrMat A(3);
  double a = M_PI/4;
  A(0,0) = cos(a);
  A(1,1) = cos(a);
  A(2,2) = 1;
  A(0,1) = sin(a);
  A(1,0) = -sin(a);
  addFrame(new FixedRelativeFrame("Q",r,A));
  group2->setFrameOfReference(getFrame("Q"));
  addGroup(group2);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
  setPlotFeatureRecursive(deflection,enabled);
}

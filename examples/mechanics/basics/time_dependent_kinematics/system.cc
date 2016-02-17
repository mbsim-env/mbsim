#include "system.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/nested_function.h"
#include "mbsim/observers/kinematics_observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cube.h"
#include "openmbvcppinterface/coilspring.h"
#endif

using namespace MBSim;
using namespace casadi;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);

  // Parameter der Körper
  double m1 = 3;
  SymMat Theta1(3,EYE);
  double h1 = 2;
  double freq1 = M_PI;
  double freq2 = M_PI/3;
  double v0y = 1;

  RigidBody *body1 = new RigidBody("Box1");
  addObject(body1);

  body1->setMass(m1);
  body1->setInertiaTensor(Theta1);

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));

  SX t=SX::sym("t");
  SX fexp=SX::zeros(3);
  fexp[0] = sin(freq1*t + M_PI/2);
  fexp[1] = v0y*t; 
  fexp[2] = 0; 
  SXFunction foo(t,fexp);

  body1->setTranslation(new SymbolicFunction<Vec3(double)>(foo));

  SX fexp2 = 5*sin(freq2*t);
  SXFunction foo2(t,fexp2);

  SymbolicFunction<double(double)> *f2 = new SymbolicFunction<double(double)>(foo2);
  body1->setRotation(new NestedFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>("[0;0;1]"), f2));
  body1->setTranslationDependentRotation(true);

  body1->getFrame("C")->setPlotFeature(globalPosition,enabled);
  body1->getFrame("C")->setPlotFeature(globalVelocity,enabled);
  body1->getFrame("C")->setPlotFeature(globalAcceleration,enabled);

  AbsoluteKinematicsObserver *o = new AbsoluteKinematicsObserver("Observer");
  addObserver(o);
  o->setFrame(body1->getFrame("C"));
  //arrow->setDiffuseColor(1/3.0, 1, 1);
  o->enableOpenMBVVelocity();

  //arrow->setDiffuseColor(0.4, 1, 1);
  o->enableOpenMBVAngularVelocity();

#ifdef HAVE_OPENMBVCPPINTERFACE
  // ----------------------- Visualisierung in OpenMBV --------------------  
  boost::shared_ptr<OpenMBV::Cube> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  cuboid->setLength(h1);
  cuboid->setDiffuseColor(240./360.,1,1);
  body1->setOpenMBVRigidBody(cuboid);

#endif
  // Just to have somtething to integrate ;-)
  RigidBody *body2 = new RigidBody("Rod2");
  addObject(body2);
  body2->setFrameOfReference(getFrame("I"));
  body2->setFrameForKinematics(body2->getFrame("C"));
  body2->setMass(1);
  body2->setInertiaTensor(SymMat3(EYE));
  body2->setTranslation(new LinearTranslation<VecV>("[0; 1; 0]"));

}

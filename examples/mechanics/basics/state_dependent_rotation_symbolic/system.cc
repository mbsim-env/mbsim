#include "system.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/composite_function.h"
#include "mbsim/links/link.h"
#include "mbsim/observers/frame_observer.h"
#include "mbsim/observers/rigid_body_observer.h"

#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cuboid.h"

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
  double m = 1;
  double l = 2;
  double h = 1;
  double d = 2;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);

  RigidBody *body = new RigidBody("Box1");
  addObject(body);

  body->setMass(m);
  body->setInertiaTensor(Theta);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));

  double R = 10;
  SX sq=SX::sym("q");

  SX pos=SX::zeros(3);
  pos(0) = -cos(sq(0)/R)*R;
  pos(1) = -sin(sq(0)/R)*R;
  pos(2) = 0;

  body->setTranslation(new SymbolicFunction<Vec3(VecV)>(pos, sq));

  SX al=M_PI/2+sq(0)/R;

  SymbolicFunction<double(VecV)> *angle = new SymbolicFunction<double(VecV)>(al, sq);
  body->setRotation(new CompositeFunction<RotMat3(double(VecV))>(new RotationAboutFixedAxis<double>("[0;0;1]"), angle));
  body->setTranslationDependentRotation(true);
  
  body->getFrame("C")->setPlotFeature(position, true);
  body->getFrame("C")->setPlotFeature(MBSim::angle, true);
  body->getFrame("C")->setPlotFeature(velocity, true);
  body->getFrame("C")->setPlotFeature(angularVelocity, true);
  body->getFrame("C")->setPlotFeature(acceleration, true);
  body->getFrame("C")->setPlotFeature(angularAcceleration, true);

  FrameObserver *o = new FrameObserver("AKObserver");
  addObserver(o);
  o->setFrame(body->getFrame("C"));
  o->enableOpenMBVVelocity();
  o->enableOpenMBVAngularVelocity();

  // ----------------------- Visualisierung in OpenMBV --------------------  
  std::shared_ptr<OpenMBV::Cuboid> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  cuboid->setLength(l,h,d);
  cuboid->setDiffuseColor(160./360.,1,1);
  body->setOpenMBVRigidBody(cuboid);

  getFrame("I")->enableOpenMBV();

  RigidBodyObserver *observer = new RigidBodyObserver("RBObserver");
  addObserver(observer);
  observer->setRigidBody(body);
  observer->enableOpenMBVWeight();
  observer->enableOpenMBVJointForce();
  observer->enableOpenMBVJointMoment();

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
}

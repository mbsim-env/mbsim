#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/symbolic_function.h"
#include "mbsim/observers/frame_observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cuboid.h"
#endif

using namespace MBSim;
using namespace CasADi;
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
  vector<SX> sq(1);
  sq[0] = SX("q1");

  vector<SX> pos(3);
  pos[0] = -cos(sq[0]/R)*R;
  pos[1] = -sin(sq[0]/R)*R;
  pos[2] = 0; 

  SXFunction spos(sq,pos);
  
  SymbolicFunction1<Vec3,Vec> *position = new SymbolicFunction1<Vec3,Vec>(spos);
  StateDependentTranslation *trans = new StateDependentTranslation(1,position);
  body->setTranslation(trans);

  vector<SX> al(1);
  al[0] = M_PI/2+sq[0]/R;

  SXFunction sangle(sq,al);
  SymbolicFunction1<double,Vec> *angle = new SymbolicFunction1<double,Vec>(sangle);
  StateDependentRotationAboutFixedAxis *rot = new StateDependentRotationAboutFixedAxis(1,angle,"[0;0;1]");
  body->setRotation(rot);
  
  body->getFrame("C")->setPlotFeature(globalPosition,enabled);
  body->getFrame("C")->setPlotFeature(globalVelocity,enabled);
  body->getFrame("C")->setPlotFeature(globalAcceleration,enabled);

  body->setOpenMBVWeightArrow(new OpenMBV::Arrow);
  body->setOpenMBVJointForceArrow(new OpenMBV::Arrow);
  body->setOpenMBVJointMomentArrow(new OpenMBV::Arrow);
  FrameObserver *o = new FrameObserver("Observer");
  addObserver(o);
  o->setFrame(body->getFrame("C"));
  OpenMBV::Arrow *arrow = new OpenMBV::Arrow;
  arrow->setReferencePoint(OpenMBV::Arrow::fromPoint);
  arrow->setStaticColor(0.5);
  o->setOpenMBVVelocityArrow(arrow);

  arrow = new OpenMBV::Arrow;
  arrow->setReferencePoint(OpenMBV::Arrow::fromPoint);
  arrow->setType(OpenMBV::Arrow::toDoubleHead);
  arrow->setStaticColor(0.4);
  o->setOpenMBVAngularVelocityArrow(arrow);

#ifdef HAVE_OPENMBVCPPINTERFACE
  // ----------------------- Visualisierung in OpenMBV --------------------  
  OpenMBV::Cuboid *cuboid=new OpenMBV::Cuboid;
  cuboid->setLength(l,h,d);
  cuboid->setStaticColor(0.0);
  body->setOpenMBVRigidBody(cuboid);

  getFrame("I")->enableOpenMBV();

#endif

}

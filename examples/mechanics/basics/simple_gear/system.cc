#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/basic_functions.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/rigid_body.h"
#include "mbsim/constraint.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/frame.h"
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

Gear::Gear(const string &projectName) : DynamicSystemSolver(projectName) {
  double r1 = 0.02;
#ifdef HAVE_OPENMBVCPPINTERFACE
  double r2 = 0.02;
  double r3 = 0.02;
#endif
  double R1 = 0.02*2;
  double R2a = 0.04*2;
  double R2b = 0.02*2;
  double R3 = 0.04*2;
  double m1 = 1;
  double J = 0.5*m1*r1*r1; 
  SymMat Theta(3);
  double l = 0.1;

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);


  RigidBody* shaft1 = new RigidBody("Shaft1");
  addObject(shaft1);
  shaft1->setFrameOfReference(getFrame("I"));
  shaft1->setFrameForKinematics(shaft1->getFrame("C"));
  shaft1->setMass(m1);
  Theta(2,2) = J;
  shaft1->setInertiaTensor(Theta);
  shaft1->setRotation(new RotationAboutZAxis<VecV>);
#ifdef HAVE_OPENMBVCPPINTERFACE
  shaft1->getFrame("C")->enableOpenMBV(0.2);
#endif
  //shaft1->setInitialGeneralizedVelocity("[1]");

  Vec r(3);
  r(1) = R1+R2a;
  r(2) = l;
  addFrame(new FixedRelativeFrame("Q",r,SqrMat(3,EYE)));

  RigidBody* shaft2 = new RigidBody("Shaft2");
  addObject(shaft2);
  shaft2->setFrameOfReference(getFrame("Q"));
  shaft2->setFrameForKinematics(shaft2->getFrame("C"));
  shaft2->setMass(m1);
  Theta(2,2) = J;
  shaft2->setInertiaTensor(Theta);
  shaft2->setRotation(new RotationAboutZAxis<VecV>);
#ifdef HAVE_OPENMBVCPPINTERFACE
  shaft2->getFrame("C")->enableOpenMBV(0.2);
#endif

  r(1) = R1+R2a-R2b-R3;
  r(2) = 2*l;
  addFrame(new FixedRelativeFrame("P",r,SqrMat(3,EYE)));
  RigidBody* shaft3 = new RigidBody("Shaft3");
  addObject(shaft3);
  shaft3->setFrameOfReference(getFrame("P"));
  shaft3->setFrameForKinematics(shaft3->getFrame("C"));
  shaft3->setMass(m1);
  Theta(2,2) = J;
  shaft3->setInertiaTensor(Theta);
  shaft3->setRotation(new RotationAboutZAxis<VecV>);
#ifdef HAVE_OPENMBVCPPINTERFACE
  shaft3->getFrame("C")->enableOpenMBV(0.2);
#endif

  GearConstraint *constraint = new GearConstraint("C1");
  addConstraint(constraint);
  constraint->setDependentBody(shaft2);
  constraint->addTransmission(Transmission(shaft1,-R1/R2a));

  constraint = new GearConstraint("C2");
  addConstraint(constraint);
  constraint->setDependentBody(shaft3);
  constraint->addTransmission(Transmission(shaft2,-R2b/R3));

  KineticExcitation* ke;
  ke = new KineticExcitation("MAn");
  addLink(ke);
  ke->connect(shaft1->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(1.1/100.));

  ke = new KineticExcitation("MAbL");
  addLink(ke);
  ke->connect(shaft3->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(-4.0/100.));

#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Frustum> c1=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c1->setTopRadius(r1);
  c1->setBaseRadius(r1);
  c1->setHeight(l);
  c1->setInitialTranslation(0,0,l/2);
  c1->setName("frustum1");

  boost::shared_ptr<OpenMBV::Frustum> c2=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c2->setTopRadius(R1);
  c2->setBaseRadius(R1);
  c2->setHeight(l/10);
  c2->setInitialTranslation(0,0,l/2+l/20);
  c2->setName("frustum2");

  boost::shared_ptr<OpenMBV::CompoundRigidBody> c = OpenMBV::ObjectFactory::create<OpenMBV::CompoundRigidBody>();
  c->addRigidBody(c1);
  c->addRigidBody(c2);
  c->setDiffuseColor(0.3333,1,0.3333);
  shaft1->setOpenMBVRigidBody(c);

  c1=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c1->setTopRadius(r2);
  c1->setBaseRadius(r2);
  c1->setHeight(l);
  c1->setInitialTranslation(0,0,l/2);
  c1->setName("frustum1");

  c2=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c2->setTopRadius(R2a);
  c2->setBaseRadius(R2a);
  c2->setHeight(l/10);
  c2->setInitialTranslation(0,0,-l/2+l/20);
  c2->setName("frustum2");

  boost::shared_ptr<OpenMBV::Frustum> c3=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c3->setTopRadius(R2b);
  c3->setBaseRadius(R2b);
  c3->setHeight(l/10);
  c3->setInitialTranslation(0,0,l/2+l/20);
  c3->setName("frustum3");

  c = OpenMBV::ObjectFactory::create<OpenMBV::CompoundRigidBody>();
  c->addRigidBody(c1);
  c->addRigidBody(c2);
  c->addRigidBody(c3);
  c->setDiffuseColor(0.6666,1,0.6666);
  shaft2->setOpenMBVRigidBody(c);

  c1=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c1->setTopRadius(r3);
  c1->setBaseRadius(r3);
  c1->setHeight(l);
  c1->setInitialTranslation(0,0,l/2);
  c1->setName("frustum1");

  c2=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  c2->setTopRadius(R3);
  c2->setBaseRadius(R3);
  c2->setHeight(l/10);
  c2->setInitialTranslation(0,0,-l/2+l/20);
  c2->setDiffuseColor(0.1111,1,1);
  c2->setName("frustum2");

  c = OpenMBV::ObjectFactory::create<OpenMBV::CompoundRigidBody>();
  c->addRigidBody(c1);
  c->addRigidBody(c2);
  c->setDiffuseColor(0.1111,1,1);
  shaft3->setOpenMBVRigidBody(c);
#endif
}


#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double mStab = 0.5;
  double lStab = 1;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = 0.2; 
  double b1 = 0.45; 
  double a2 = 0.2; 
  double phi1 = -M_PI/2;
  double phi2 = -M_PI/4;

  Vec WrOK(3);
  Vec KrKS(3);
  SymMat Theta(3);

  RigidBody* stab1 = new RigidBody("Stab1");
  addObject(stab1);
  KrKS(1) = a1;

  stab1->addFrame("Ref",KrKS,SqrMat(3,EYE));

  stab1->setFrameOfReference(getFrame("I"));
  stab1->setFrameForKinematics(stab1->getFrame("Ref"));

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  Vec q0(1);
  q0(0) = -phi1;
  stab1->setInitialGeneralizedPosition(q0);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum * cylinder = new OpenMBV::Frustum;
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(lStab);
  cylinder->setStaticColor(0.5);
  stab1->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);
#endif

  RigidBody* stab2 = new RigidBody("Stab2");
  addObject(stab2);
  KrKS(1) = -b1;
  stab1->addFrame("P",KrKS,SqrMat(3,EYE));
  KrKS(1) = a2;
  stab2->addFrame("R",KrKS,SqrMat(3,EYE));
  stab2->setFrameOfReference(stab1->getFrame("P"));
  stab2->setFrameForKinematics(stab2->getFrame("R"));
  stab2->setMass(mStab);
  Theta(2,2) = JStab;
  stab2->setInertiaTensor(Theta,stab2->getFrame("C"));
  stab2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  q0(0) = -phi2+phi1;
  stab2->setInitialGeneralizedPosition(q0);

  stab1->setForceDirection("[1,0;0,1;0,0]]");
  stab2->setForceDirection("[1,0;0,1;0,0]]");

#ifdef HAVE_OPENMBVCPPINTERFACE
  cylinder = new OpenMBV::Frustum;
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(lStab);
  cylinder->setStaticColor(0.1);
  stab2->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);
#endif

}


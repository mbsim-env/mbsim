#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "test_group.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"

#include "openmbvcppinterface/ivbody.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

Pendulum::Pendulum(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double mStab = 0.2;
  double lStab = 0.3;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = 0.15*lStab;
  double a2 = 0.15*lStab;

  Vec KrRP(3), KrCR(3);
  SymMat Theta(3);

  RigidBody* stab1 = new RigidBody("Stab1");
  addObject(stab1);
  KrCR(0) = a1;

  stab1->addFrame(new FixedRelativeFrame("R",KrCR,SqrMat(3,EYE)));

  stab1->setFrameOfReference(getFrame("I"));
  stab1->setFrameForKinematics(stab1->getFrame("R"));

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));

  std::shared_ptr<OpenMBV::IvBody> obj=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  obj->setIvFileName("objects/pendel1.wrl");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  stab1->setOpenMBVRigidBody(obj);
  stab1->setOpenMBVFrameOfReference(stab1->getFrame("R"));

  RigidBody* stab2 = new RigidBody("Stab2");
  addObject(stab2);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  stab1->addFrame(new FixedRelativeFrame("P",KrRP,SqrMat(3,EYE),stab1->getFrame("R")));
  KrCR(0) = a2;
  stab2->addFrame(new FixedRelativeFrame("R",-KrCR,SqrMat(3,EYE)));
  stab2->setFrameOfReference(stab1->getFrame("P"));
  stab2->setFrameForKinematics(stab2->getFrame("R"));
  stab2->setMass(mStab);
  Theta(2,2) = JStab;
  stab2->setInertiaTensor(Theta);
  stab2->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  stab2->setGeneralizedInitialPosition(Vec("[-1.6]"));

  obj=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  obj->setIvFileName("objects/pendel2.wrl");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  stab2->setOpenMBVRigidBody(obj);
  stab2->setOpenMBVFrameOfReference(stab2->getFrame("R"));

  RigidBody* stab3 = new RigidBody("Stab3");
  addObject(stab3);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  stab2->addFrame(new FixedRelativeFrame("P",KrRP,SqrMat(3,EYE),stab2->getFrame("R")));
  KrCR(0) = a2;
  stab3->addFrame(new FixedRelativeFrame("R",-KrCR,SqrMat(3,EYE)));
  stab3->setFrameOfReference(stab2->getFrame("P"));
  stab3->setFrameForKinematics(stab3->getFrame("R"));
  stab3->setMass(mStab);
  Theta(2,2) = JStab;
  stab3->setInertiaTensor(Theta);
  stab3->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  stab3->setGeneralizedInitialPosition(Vec("[-1.6]"));

  obj=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  obj->setIvFileName("objects/pendel2.wrl");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  stab3->setOpenMBVRigidBody(obj);
  stab3->setOpenMBVFrameOfReference(stab3->getFrame("R"));

  RigidBody* stab4 = new RigidBody("Stab4");
  addObject(stab4);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  KrCR(0) = a2;
  stab4->setFrameOfReference(getFrame("I"));
  stab4->setFrameForKinematics(stab4->getFrame("C"));
  stab4->setMass(mStab);
  Theta(2,2) = JStab;
  stab4->setInertiaTensor(Theta);
  stab4->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  stab4->setGeneralizedInitialVelocity(Vec("[-1.6]"));

  obj=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  obj->setIvFileName("objects/pendel2.wrl");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  stab4->setOpenMBVRigidBody(obj);

  TestGroup *group = new TestGroup("PendelGruppe1"); 
  addGroup(group);
  stab3->addFrame(new FixedRelativeFrame("P",KrRP,SqrMat(3,EYE),stab3->getFrame("R")));
  group->getRod1()->setFrameOfReference(stab3->getFrame("P"));

  group = new TestGroup("PendelGruppe2"); 
  Vec r(3);
  r(0) = 0.2;
  addFrame(new FixedRelativeFrame("Q",r));
  group->setFrameOfReference(getFrame("Q"));
  addGroup(group);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
}

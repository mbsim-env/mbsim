#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/special_classes.h"
#include "test_group.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/objbody.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

Pendulum::Pendulum(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);

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

  stab1->addFrame("R",KrCR,SqrMat(3,EYE));

  stab1->setFrameOfReference(getFrame("I"));
  stab1->setFrameForKinematics(stab1->getFrame("R"));

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

#if HAVE_OPENMBVCPPINTERFACE
  OpenMBV::ObjBody *obj=new OpenMBV::ObjBody;
  obj->setObjFileName("objects/pendel1.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(OpenMBV::ObjBody::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab1->setOpenMBVRigidBody(obj);
#endif

  RigidBody* stab2 = new RigidBody("Stab2");
  addObject(stab2);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  stab1->addFrame("P",KrRP,SqrMat(3,EYE),stab1->getFrame("R"));
  KrCR(0) = a2;
  stab2->addFrame("R",-KrCR,SqrMat(3,EYE));
  stab2->setFrameOfReference(stab1->getFrame("P"));
  stab2->setFrameForKinematics(stab2->getFrame("R"));
  stab2->setMass(mStab);
  Theta(2,2) = JStab;
  stab2->setInertiaTensor(Theta,stab2->getFrame("C"));
  stab2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  stab2->setq0(Vec("[-1.6]"));

#if HAVE_OPENMBVCPPINTERFACE
  obj=new OpenMBV::ObjBody;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(OpenMBV::ObjBody::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab2->setOpenMBVRigidBody(obj);
#endif

  RigidBody* stab3 = new RigidBody("Stab3");
  addObject(stab3);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  stab2->addFrame("P",KrRP,SqrMat(3,EYE),stab2->getFrame("R"));
  KrCR(0) = a2;
  stab3->addFrame("R",-KrCR,SqrMat(3,EYE));
  stab3->setFrameOfReference(stab2->getFrame("P"));
  stab3->setFrameForKinematics(stab3->getFrame("R"));
  stab3->setMass(mStab);
  Theta(2,2) = JStab;
  stab3->setInertiaTensor(Theta,stab3->getFrame("C"));
  stab3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  stab3->setq0(Vec("[-1.6]"));

#if HAVE_OPENMBVCPPINTERFACE
  obj=new OpenMBV::ObjBody;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(OpenMBV::ObjBody::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab3->setOpenMBVRigidBody(obj);
#endif

  RigidBody* stab4 = new RigidBody("Stab4");
  addObject(stab4);
  KrRP(0) = lStab/2;
  KrRP(2) = 0.006;
  KrCR(0) = a2;
  stab4->setFrameOfReference(getFrame("I"));
  stab4->setFrameForKinematics(stab4->getFrame("C"));
  stab4->setMass(mStab);
  Theta(2,2) = JStab;
  stab4->setInertiaTensor(Theta,stab4->getFrame("C"));
  stab4->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  stab4->setu0(Vec("[-1.6]"));

#if HAVE_OPENMBVCPPINTERFACE
  obj=new OpenMBV::ObjBody;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(OpenMBV::ObjBody::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab4->setOpenMBVRigidBody(obj);
#endif

  TestGroup *group = new TestGroup("PendelGruppe1"); 
  addDynamicSystem(group);
  stab3->addFrame("P",KrRP,SqrMat(3,EYE),stab3->getFrame("R"));
  group->getRod1()->setFrameOfReference(stab3->getFrame("P"));

  group = new TestGroup("PendelGruppe2"); 
  Vec r(3);
  r(0) = 0.2;
  group->setPosition(r);
  addDynamicSystem(group);
  KrRP(0) = lStab/2;
  KrRP(2) = -0.006;
}


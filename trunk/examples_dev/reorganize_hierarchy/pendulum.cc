#include "pendulum.h"
#include "mbsim/rigid_body.h"
#include "special_classes.h"
#include "test_group.h"

#ifdef HAVE_AMVIS
#include "objobject.h"

using namespace AMVis;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include "amviscppinterface/objobject.h"
#endif

Pendulum::Pendulum(const string &projectName) : MultiBodySystem(projectName) {

  setProjectDirectory("plot");

  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);

  SpecialGroup *mbs = new SpecialGroup("MehrfachPendel"); 
  addSubsystem(mbs,Vec(3),SqrMat(3,EYE));

  double mStab = 0.2;
  double lStab = 0.3;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = 0.15*lStab;
  double a2 = 0.15*lStab;

  Vec KrRP(3), KrCR(3);
  SymMat Theta(3);

  SpecialRigidBody* stab1 = new SpecialRigidBody("Stab1");
  mbs->addObject(stab1);
  //Node* node = mbs->addObject(0,stab1);
  KrCR(0) = a1;

  stab1->addFrame("R",KrCR,SqrMat(3,EYE));

  stab1->setFrameOfReference(getFrame("I"));
  stab1->setFrameForKinematics(stab1->getFrame("R"));

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  AMVis::ObjObject *obj=new AMVis::ObjObject;
  obj->setObjFileName("objects/pendel1.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(AMVis::ObjObject::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab1->setAMVisRigidBody(obj);

  SpecialRigidBody* stab2 = new SpecialRigidBody("Stab2");
  mbs->addObject(stab2);
  //mbs->addObject(node,stab2);
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

  obj=new AMVis::ObjObject;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(AMVis::ObjObject::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab2->setAMVisRigidBody(obj);

  SpecialRigidBody* stab3 = new SpecialRigidBody("Stab3");
  mbs->addObject(stab3);
  //mbs->addObject(node,stab3);
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

  obj=new AMVis::ObjObject;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(AMVis::ObjObject::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab3->setAMVisRigidBody(obj);

  SpecialRigidBody* stab4 = new SpecialRigidBody("Stab4");
  mbs->addObject(stab4);
  //mbs->addObject(node,stab4);
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

  obj=new AMVis::ObjObject;
  obj->setObjFileName("objects/pendel2.obj");
  obj->setScaleFactor(0.1*0.3);
  obj->setInitialRotation(0,0,M_PI/2);
  obj->setNormals(AMVis::ObjObject::smoothIfLessBarrier);
  obj->setEpsVertex(1e-5);
  obj->setEpsNormal(1e-5);
  obj->setSmoothBarrier(M_PI*2/9);
  stab4->setAMVisRigidBody(obj);

  TestGroup *group = new TestGroup("PendelGruppe1"); 
  mbs->addSubsystem(group,Vec(3),SqrMat(3,EYE));
  stab3->addFrame("P",KrRP,SqrMat(3,EYE),stab3->getFrame("R"));
  group->getRod1()->setFrameOfReference(stab3->getFrame("P"));

  group = new TestGroup("PendelGruppe2"); 
  Vec r(3);
  r(0) = 0.2;
  mbs->addSubsystem(group,r,SqrMat(3,EYE));
  KrRP(0) = lStab/2;
  KrRP(2) = -0.006;
}


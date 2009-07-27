#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/objbody.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

Pendulum::Pendulum(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  Tree *tree = new Tree("Baum"); 
  addDynamicSystem(tree);

  double mStab = 0.2;
  double lStab = 0.3;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = 0.15*lStab;
  double a2 = 0.15*lStab;

  Vec KrRP(3), KrCR(3);
  SymMat Theta(3);

  RigidBody* stab1 = new RigidBody("Stab1");
  Node* node = tree->addObject(0,stab1);
  KrCR(0) = a1;

  stab1->addFrame("R",KrCR,SqrMat(3,EYE));

  stab1->setFrameOfReference(getFrame("I"));
  stab1->setFrameForKinematics(stab1->getFrame("R"));

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

#ifdef HAVE_OPENMBVCPPINTERFACE
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
  tree->addObject(node,stab2);
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
  stab2->setInitialGeneralizedPosition(Vec("[-1.6]"));

#ifdef HAVE_OPENMBVCPPINTERFACE
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

}


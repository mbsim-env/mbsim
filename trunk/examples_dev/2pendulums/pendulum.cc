#include "pendulum.h"
#include "rigid_body.h"
#include "objobject.h"

using namespace AMVis;

Pendulum::Pendulum(const string &projectName) : Tree(projectName) {

  double mStab = 0.2;
  double lStab = 0.3;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = -0.15*lStab;
  double a2 = 0.15*lStab;

  Vec WrOK(3);
  Vec KrKS(3);
  SymMat Theta(3);

  stab1 = new RigidBody("Stab1");
  Node* node = addObject(0,stab1);
  KrKS(0) = a1;
  SqrMat A(3);
  for(int i=0; i<3; i++)
    A(i,i) = 1;


  stab1->addCoordinateSystem("Ref",-KrKS,A);
  stab1->setCoordinateSystemForKinematics(stab1->getCoordinateSystem("Ref"));
  stab1->setFrameOfReference(getCoordinateSystem("I"));

  stab1->setqSize(1);
  stab1->setuSize(1);

  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertiaTensor(Theta);
  stab1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  ObjObject * obj = new ObjObject(name+stab1->getName(),1,false);
  obj->setObjFilename("objects/pendel1.obj");
  stab1->setAMVisBody(obj);
  obj->setScaleFactor(0.1*0.3);
  obj -> setInitialRotation(0,0,M_PI/2);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  if(1) {
  stab2 = new RigidBody("Stab2");
  WrOK(0) = lStab/2;
  WrOK(2) = 0.006;
  stab1->addCoordinateSystem("P",WrOK-KrKS,A);
  KrKS(0) = a2;
  stab2->addCoordinateSystem("R",-KrKS,A);
  addObject(node,stab2);
  stab2->setqSize(1);
  stab2->setuSize(1);
  stab2->setFrameOfReference(stab1->getCoordinateSystem("P"));
  stab2->setCoordinateSystemForKinematics(stab2->getCoordinateSystem("R"));
  stab2->setMass(mStab);
  Theta(2,2) = JStab;
  stab2->setInertiaTensor(Theta,stab2->getCoordinateSystem("C"));
  stab2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  stab2->setq0(Vec("[-1.6]"));

  obj = new ObjObject(name+stab2->getName(),1,false);
  obj->setObjFilename("objects/pendel2.obj");
  stab2->setAMVisBody(obj);
  obj->setScaleFactor(0.1*0.3);
  obj -> setInitialRotation(0,0,M_PI/2);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);
  }

}


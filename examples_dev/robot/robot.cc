#include "robot.h"
#include "body_rigid.h"
#include "userfunction.h"
#include "connection_flexible.h"
#include "load.h"
#include "cuboid.h"
#include "cylinder.h"
#include "objobject.h"
//#include "transfersys.h"
#include "tree.h"

using namespace AMVis;

Robot::Robot(const string &projectName) : MultiBodySystem(projectName) {

  // Gravitation
  Vec grav(3);
  grav(1)=-1;
  grav(2)=-1;
  grav=grav/nrm2(grav)*9.81;
  setAccelerationOfGravity(grav);

  // Data
  double mB = 20;
  double mA = 10;
  double mS = 5;
  double hB= 1;
  double lA= 1;
  double rB= 0.2;
  double rA= 0.1;
  double rS= 0.05;

  // --------------------------- Setup MBS ----------------------------
  
  // System with tree-structure
  Tree *tree = new Tree("Baum");
  addObject(tree);

  BodyRigid *basis = new BodyRigid("Basis");
  tree->addObject(basis);
  basis->setMass(mB);
  SymMat Theta(3);
  Theta(0,0) = mB*rB*rB;
  Theta(1,1) = 1./2.*mB*rB*rB;
  Theta(2,2) = mB*rB*rB;
  basis->setInertia(Theta);

  SqrMat A(3);
  for(int i=0; i<3; i++)
    A(i,i) = 1;


  Vec KrKS(3);
  KrKS(1) = hB/2;
  basis->setfAPK(new RotationAxis(Vec("[0;1;0]")));
  Vec KrSP(3);
  KrSP(1) = hB/2;
  basis->addCoordinateSystem("R",-KrKS,A);
  basis->setParentCoordinateSystem(getCoordinateSystem("O"));
  basis->setRefCoordinateSystem(basis->getCoordinateSystem("R"));

  BodyRigid *arm = new BodyRigid("Arm");
  tree->addObject(arm);
  Vec PrPK0(3);
  PrPK0(1) = hB;
  basis->addCoordinateSystem("P",PrPK0,A,basis->getCoordinateSystem("R"));
  KrKS.init(0);
  KrKS(1) = lA/2;
  arm->addCoordinateSystem("R",-KrKS,A);
  arm->setParentCoordinateSystem(basis->getCoordinateSystem("P"));
  arm->setRefCoordinateSystem(arm->getCoordinateSystem("R"));
  arm->setq0(Vec("[0.3]"));

  arm->setMass(mA);
  Theta(0,0) = mA*rA*rA;
  Theta(1,1) = 1./2.*mA*rA*rA;
  Theta(2,2) = mA*rA*rA;
  arm->setInertia(Theta);
  arm->setfAPK(new RotationAxis(Vec("[0;0;1]")));
  KrSP(1) = -lA/2;
  KrSP(1) = lA/2;

  BodyRigid *spitze = new BodyRigid("Spitze");
  tree->addObject(spitze);
  PrPK0.init(0);
  PrPK0(1) = lA;
  arm->addCoordinateSystem("P",PrPK0,A,arm->getCoordinateSystem("R"));
  spitze->setMass(mS);
  Theta(0,0) = mS*rS*rS;
  Theta(1,1) = 1./2.*mS*rS*rS;
  Theta(2,2) = mS*rS*rS;
  spitze->setInertia(Theta);
  spitze->setfPrPK(new LinearTranslation(Vec("[0;1;0]")));
  spitze->setParentCoordinateSystem(arm->getCoordinateSystem("P"));
  spitze->setRefCoordinateSystem(spitze->getCoordinateSystem("COG"));
  
  // --------------------------- Setup Visualisation ----------------------------
  ObjObject *obj = new ObjObject(basis->getName(),1,false);
  obj->setObjFilename("objects/basis.obj");
  basis->setAMVisBody(obj);
  obj->setScaleFactor(0.2);
  obj -> setInitialTranslation(0,0.25,0);
  obj -> setInitialRotation(-M_PI/2,0,0);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  obj = new ObjObject(arm->getName(),1,false);
  obj->setObjFilename("objects/arm.obj");
  arm->setAMVisBody(obj);
  obj->setScaleFactor(0.2);
  obj -> setInitialTranslation(0,0.08,0);
  obj -> setInitialRotation(-M_PI/2,0,0);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  obj = new ObjObject(spitze->getName(),1,false);
  obj->setObjFilename("objects/spitze.obj");
  spitze->setAMVisBody(obj);
  obj->setScaleFactor(0.2);
  obj -> setInitialTranslation(0,-0.3,0);
  obj -> setInitialRotation(-M_PI/2,0,0);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);
}

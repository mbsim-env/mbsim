#include "pendulum.h"
#include "body_rigid_rel.h"
#include "objobject.h"
#include "tree_rigid.h"

using namespace AMVis;

Pendulum::Pendulum(const string &projectName) : MultiBodySystem(projectName) {

  setProjectDirectory("plot");
  setPlotLevel(3); 

  Vec grav(3);
  grav(1)=-9.81;
  setGrav(grav);

  TreeRigid *tree = new TreeRigid("Baum"); 
  addObject(tree);

  double mStab = 0.2;
  double lStab = 0.3;
  double JStab = 1.0/12.0 * mStab * lStab * lStab; 
  double a1 = -0.15*lStab;
  double a2 = 0.15*lStab;

  Vec WrOK(3);
  Vec KrKS(3);
  SymMat Theta(3);

  BodyRigidRel* stab1 = new BodyRigidRel("Stab1");
  tree->setRoot(stab1);
  stab1->setPrPK0(WrOK);
  KrKS(0) = a1;
  stab1->setKrKS(KrKS);
  stab1->setMass(mStab);
  Theta(2,2) = JStab;
  stab1->setInertia(Theta,true);
  stab1->setJR("[0; 0; 1]");

  BodyRigidRel* stab2 = new BodyRigidRel("Stab2");
  stab1->addChild(stab2);
  stab2->setPrPK0(WrOK);
  KrKS(0) = a2;
  stab2->setKrKS(KrKS);
  stab2->setMass(mStab);
  Theta(2,2) = JStab;
  stab2->setInertia(Theta,true);
  stab2->setJR("[0; 0; 1]");
  WrOK(0) = lStab/2;
  WrOK(2) = 0.006;
  stab2->setPrPK0(WrOK);
  stab2->setq0(Vec("[-1.6]"));

  ObjObject * obj = new ObjObject(stab1->getFullName(),1,false);
  obj->setObjFilename("objects/pendel1.obj");
  stab1->setAMVisBody(obj);
  obj->setScaleFactor(0.1*0.3);
  obj -> setInitialRotation(0,0,M_PI/2);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  obj = new ObjObject(stab2->getFullName(),1,false);
  obj->setObjFilename("objects/pendel2.obj");
  stab2->setAMVisBody(obj);
  obj->setScaleFactor(0.1*0.3);
  obj -> setInitialRotation(0,0,M_PI/2);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  stab1->setPlotLevel(3);
  stab2->setPlotLevel(3);
  setPlotLevel(3);
}


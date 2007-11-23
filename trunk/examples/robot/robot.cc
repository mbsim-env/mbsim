#include "robot.h"
#include "body_rigid.h"
#include "body_rigid_rel.h"
#include "userfunction.h"
#include "connection_flexible.h"
#include "load.h"
#include "cuboid.h"
#include "cylinder.h"
#include "objobject.h"
#include "transfersys.h"
#include "tree_rigid.h"

using namespace AMVis;

class Pos : public UserFunction {
  private:
    double T1, T2;
    BodyRigidRel *body;
  public:
    Pos(BodyRigidRel *body_) : T1(2), T2(4), body(body_) {}
    
    Vec operator()(double t) {
      Vec pos(1);
      pos(0) = body->getq()(0);
      return pos;
    };
};


Robot::Robot(const string &projectName) : MultiBodySystem(projectName) {

  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setGrav(grav);

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
  TreeRigid *tree = new TreeRigid("Baum");
  addObject(tree);

  // Body with relative kinematics
  BodyRigidRel *basis = new BodyRigidRel("Basis");
  basis->setPlotLevel(2);
  tree->setRoot(basis);
  basis->setMass(mB);
  SymMat Theta(3);
  Theta(0,0) = mB*rB*rB;
  Theta(1,1) = 1./2.*mB*rB*rB;
  Theta(2,2) = mB*rB*rB;
  basis->setInertia(Theta,true);

  Vec KrKS(3);
  KrKS(1) = hB/2;
  basis->setKrKS(KrKS);
  basis->addPort("COG",Vec(3));
  basis->setJR(Mat("[0; 1; 0]"));
  Vec KrSP(3);
  KrSP(1) = hB/2;
  basis->addPort("Port",KrSP); 

  // Body with relative kinematics
  BodyRigidRel *arm = new BodyRigidRel("Arm");
  basis->addChild(arm);
  Vec PrPK0(3);
  PrPK0(1) = hB;
  arm->setPrPK0(PrPK0);
  KrKS.init(0);
  KrKS(1) = lA/2;
  arm->setKrKS(KrKS);
  arm->addPort("COG",Vec(3));

  arm->setMass(mA);
  Theta(0,0) = mA*rA*rA;
  Theta(1,1) = 1./2.*mA*rA*rA;
  Theta(2,2) = mA*rA*rA;
  arm->setInertia(Theta,true);
  arm->setJR(Mat("[0; 0; 1]"));
  arm->setq0("[0]");
  KrSP(1) = -lA/2;
  arm->addPort("Port",KrSP); 
  KrSP(1) = lA/2;
  arm->addPort("Port2",KrSP); 

  // Body with relative kinematics
  BodyRigidRel *spitze = new BodyRigidRel("Spitze");
  arm->addChild(spitze);
  PrPK0.init(0);
  PrPK0(1) = lA;
  spitze->setPrPK0(PrPK0);
  spitze->addPort("COG",Vec(3));
  spitze->setMass(mS);
  Theta(0,0) = mS*rS*rS;
  Theta(1,1) = 1./2.*mS*rS*rS;
  Theta(2,2) = mS*rS*rS;
  spitze->setInertia(Theta,true);
  spitze->setJT(Mat("[0; 1; 0]"));
  
  // --------------------------- Setup Control ----------------------------

  FuncTable *basisSoll=new FuncTable;
  basisSoll->setFile("Soll_Basis.tab");   

  TransferSys *tf = new TransferSys("ReglerBasis");
  addEDI(tf);
  tf->setInSignalnWeight(new Pos(basis),-1);
  tf->setInSignalnWeight(basisSoll,1);
  tf->setPID(4000,0,200);

  Load *motorBasis = new Load("MotorBasis");
  addLink(motorBasis);
  motorBasis->setUserFunction(tf->SigOut());
  motorBasis->setMomentDirection("[0;1;0]");
  motorBasis->connect(basis->getPort("COG"));

  FuncTable *spitzeSoll=new FuncTable;
  spitzeSoll->setFile("Soll_Spitze.tab");   

  tf = new TransferSys("ReglerSpitze");
  addEDI(tf);
  tf->setInSignalnWeight(new Pos(spitze),-1);
  tf->setInSignalnWeight(spitzeSoll,1);
  tf->setPID(40000,1000,200);

  Load *motorSpitze = new Load("MotorSpitze");
  addLink(motorSpitze);
  motorSpitze->setUserFunction(tf->SigOut());
  motorSpitze->setForceDirection("[0;1;0]");
  motorSpitze->connect(spitze->getPort("COG"));
  motorSpitze->setKOSY(1);

  FuncTable *armSoll=new FuncTable;
  armSoll->setFile("Soll_Arm.tab");   

  tf = new TransferSys("ReglerArm");
  addEDI(tf);
  tf->setInSignalnWeight(new Pos(arm),-1);
  tf->setInSignalnWeight(armSoll,1);
  tf->setPID(4000,0,200);

  Load *motorArm = new Load("MotorArm");
  addLink(motorArm);
  motorArm->setUserFunction(tf->SigOut());
  motorArm->setMomentDirection("[0;0;1]");
  motorArm->connect(arm->getPort("COG"));
  motorArm->setKOSY(1);

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

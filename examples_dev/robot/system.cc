#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/userfunction.h"
#include "mbsim/load.h"
#include "mbsim/actuator.h"
#include "mbsimControl/transfersys.h"
#include "mbsim/tree.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/ivbody.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class PositionSensor : public UserFunction {
  private:
    RigidBody *body;
  public:
    PositionSensor(RigidBody *body_) : body(body_) {}

    Vec operator()(double t) {
      Vec pos(1);
      pos(0) = body->getq()(0);
      return pos;
    };
};

Robot::Robot(const string &projectName) : DynamicSystemSolver(projectName) {

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
  addDynamicSystem(tree,Vec(3),SqrMat(3,EYE));

  RigidBody *basis = new RigidBody("Basis");
  Node *node = tree->addObject(0,basis);
  basis->setMass(mB);
  SymMat Theta(3);
  Theta(0,0) = mB*rB*rB;
  Theta(1,1) = 1./2.*mB*rB*rB;
  Theta(2,2) = mB*rB*rB;
  basis->setInertiaTensor(Theta);

  SqrMat A(3);
  for(int i=0; i<3; i++)
    A(i,i) = 1;

  Vec KrKS(3);
  KrKS(1) = hB/2;
  basis->setRotation(new RotationAboutFixedAxis(Vec("[0;1;0]")));
  Vec KrSP(3);
  KrSP(1) = hB/2;
  basis->addFrame("R",-KrKS,A);
  basis->setFrameOfReference(getFrame("I"));
  basis->setFrameForKinematics(basis->getFrame("R"));

  RigidBody *arm = new RigidBody("Arm");
  node = tree->addObject(node,arm);
  Vec PrPK0(3);
  PrPK0(1) = hB;
  basis->addFrame("P",PrPK0,A,basis->getFrame("R"));
  KrKS.init(0);
  KrKS(1) = lA/2;
  arm->addFrame("R",-KrKS,A);
  arm->setFrameOfReference(basis->getFrame("P"));
  arm->setFrameForKinematics(arm->getFrame("R"));

  arm->setMass(mA);
  Theta(0,0) = mA*rA*rA;
  Theta(1,1) = 1./2.*mA*rA*rA;
  Theta(2,2) = mA*rA*rA;
  arm->setInertiaTensor(Theta);
  arm->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  KrSP(1) = -lA/2;
  KrSP(1) = lA/2;

  RigidBody *spitze = new RigidBody("Spitze");
  tree->addObject(node,spitze);
  spitze->setMass(mS);
  Theta(0,0) = mS*rS*rS;
  Theta(1,1) = 1./2.*mS*rS*rS;
  Theta(2,2) = mS*rS*rS;
  PrPK0(1) = lA;
  arm->addFrame("Q",PrPK0,SqrMat(3,EYE),arm->getFrame("R"));
  spitze->setInertiaTensor(Theta);
  spitze->setTranslation(new LinearTranslation(Vec("[0;1;0]")));
  spitze->setFrameOfReference(arm->getFrame("Q"));
  spitze->setFrameForKinematics(spitze->getFrame("C"));

  // --------------------------- Setup Control ----------------------------

  FuncTable *basisSoll=new FuncTable;
  basisSoll->setFile("Soll_Basis.tab");   

  TransferSys *tf = new TransferSys("ReglerBasis");
  addOrderOneDynamics(tf);
  tf->setInSignalnWeight(new PositionSensor(basis),1);
  tf->setInSignalnWeight(basisSoll,-1);
  tf->setPID(4000,0,200);

  Actuator *motorBasis = new Actuator("MotorBasis");
  addLink(motorBasis);
  motorBasis->setSignal(tf->SigOut());
  motorBasis->setMomentDirection("[0;1;0]");
  motorBasis->connect(getFrame("I"),basis->getFrame("R"));

  FuncTable *armSoll=new FuncTable;
  armSoll->setFile("Soll_Arm.tab");   

  tf = new TransferSys("ReglerArm");
  addOrderOneDynamics(tf);
  tf->setInSignalnWeight(new PositionSensor(arm),1);
  tf->setInSignalnWeight(armSoll,-1);
  tf->setPID(4000,0,200);

  Actuator *motorArm = new Actuator("MotorArm");
  addLink(motorArm);
  motorArm->setSignal(tf->SigOut());
  motorArm->setMomentDirection("[0;0;1]");
  motorArm->connect(basis->getFrame("P"),arm->getFrame("R"));

  FuncTable *spitzeSoll=new FuncTable;
  spitzeSoll->setFile("Soll_Spitze.tab");   

  tf = new TransferSys("ReglerSpitze");
  addOrderOneDynamics(tf);
  tf->setInSignalnWeight(new PositionSensor(spitze),1);
  tf->setInSignalnWeight(spitzeSoll,-1);
  tf->setPID(4000,0,200);

  Actuator *motorSpitze = new Actuator("MotorSpitze");
  addLink(motorSpitze);
  motorSpitze->setSignal(tf->SigOut());
  motorSpitze->setForceDirection("[0;1;0]");
  motorSpitze->connect(arm->getFrame("Q"),spitze->getFrame("C"));


#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::IvBody *obj=new OpenMBV::IvBody;
  obj->setIvFileName("wrl/basis.wrl");
  obj->setScaleFactor(0.2);
  obj->setInitialRotation(M_PI,0,0);
  obj->setInitialTranslation(0,0.25,0);
  basis->setOpenMBVRigidBody(obj);

  obj=new OpenMBV::IvBody;
  obj->setIvFileName("wrl/arm.wrl");
  obj->setScaleFactor(0.2);
  obj->setInitialRotation(M_PI,0,0);
  obj->setInitialTranslation(0,0.08,0);
  arm->setOpenMBVRigidBody(obj);

  obj=new OpenMBV::IvBody;
  obj->setIvFileName("wrl/spitze.wrl");
  obj->setScaleFactor(0.2);
  obj->setInitialTranslation(0,-0.3,0);
  obj->setInitialRotation(M_PI,0,0);
  spitze->setOpenMBVRigidBody(obj);
#endif
}

#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/userfunction.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/load.h"

#ifdef HAVE_AMVIS
#include "cuboid.h"
#include "cylinder.h"

using namespace AMVis;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/cylinder.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

extern bool rigidJoints;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);

  double m1 = 0.5;
  double m2 = 0.5;
  double l1 = 1;
  double l2 = 1;
  double a1 = 0.2; 
  double b1 = 0.45; 
  double a2 = 0.2; 
  double phi1 = -M_PI/2;
  double phi2 = -M_PI/4;

  RigidBody *box1 = new RigidBody("Stab1");
  addObject(box1);
  box1->setMass(m1);
  SymMat Theta(3);
  Theta(2,2) = 1./12.*m1*l1*l1;
  box1->setInertiaTensor(Theta);

  SqrMat E(3);
  E << DiagMat(3,INIT,1);
  Vec KrSP(3);
  KrSP(1) = a1;
  box1->addFrame("PunktO",KrSP,E);

  box1->setTranslation( new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  box1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  box1->setFrameOfReference(getFrame("I"));
  box1->setFrameForKinematics(box1->getFrame("C"));
  Vec q0(3);
  q0(0) = a1;
  q0(2) = -phi1;
  box1->setq0(q0);

  KrSP(1) = -b1;
  box1->addFrame("PunktU",KrSP,E);

  RigidBody *box2 = new RigidBody("Stab2");
  addObject(box2);
  box2->setMass(m2);
  Theta(2,2) = 1./12.*m2*l2*l2;
  box2->setInertiaTensor(Theta);

  KrSP(1) = a2;
  box2->addFrame("Punkt",KrSP,E);
  box2->setTranslation( new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  box2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  SqrMat A1(3);
  A1(0,0) = cos(phi1);
  A1(0,1) = sin(phi1);
  A1(1,0) = -sin(phi1);
  A1(1,1) = cos(phi1);
  A1(2,2) = 1;
  SqrMat A2(3);
  A2(0,0) = cos(phi2);
  A2(0,1) = sin(phi2);
  A2(1,0) = -sin(phi2);
  A2(1,1) = cos(phi2);
  A2(2,2) = 1;
  Vec rOP(3);
  rOP(1) = -a1-b1;
  q0 = A1*rOP + A2*(-KrSP);
  q0(2) = -phi2;
  box2->setq0(q0);

  addFrame("Os","[0;0;0.04]",E);
  box2->setFrameOfReference(getFrame("Os"));
  box2->setFrameForKinematics(box2->getFrame("C"));

  Joint *joint1 = new Joint("Gelenk1");
  addLink(joint1);
  joint1->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint1->connect(getFrame("I"),box1->getFrame("PunktO"));

  Joint *joint2 = new Joint("Gelenk2");
  addLink(joint2);
  joint2->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint2->connect(box1->getFrame("PunktU"),box2->getFrame("Punkt"));

  if(rigidJoints) {
    joint1->setForceLaw(new BilateralConstraint);
    joint1->setImpactForceLaw(new BilateralImpact);
    joint2->setForceLaw(new BilateralConstraint);
    joint2->setImpactForceLaw(new BilateralImpact);
  } 
  else {
    joint1->setForceLaw(new LinearRegularizedBilateralConstraint(1e7,1));
    joint2->setForceLaw(new LinearRegularizedBilateralConstraint(1e7,1));
  }

#ifdef HAVE_AMVIS
  ////////////////////// Visualisierung in AMVis ////////////////////////
  Cylinder * cylinder = new Cylinder(getName() + "." + box1->getName(),1,false);
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l1);
  cylinder->setColor(0.5);
  box1->setAMVisBody(cylinder);

  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);
  cylinder = new Cylinder(getName() + "." + box2->getName(),1,false);
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l2);
  cylinder->setColor(0.1);
  box2->setAMVisBody(cylinder);
  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);
#endif
#ifdef HAVE_AMVISCPPINTERFACE
  AMVis::Cylinder *cylinder=new AMVis::Cylinder;
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l1);
  cylinder->setInitialTranslation(0,-0.5,0);
  cylinder->setInitialRotation(1.5708,0,0);
  box1->setAMVisRigidBody(cylinder);

  cylinder=new AMVis::Cylinder;
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l2);
  cylinder->setInitialTranslation(0,-0.5,0);
  cylinder->setInitialRotation(1.5708,0,0);
  box2->setAMVisRigidBody(cylinder);
#endif
}


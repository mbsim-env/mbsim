#include "system.h"
#include "rigid_body.h"
#include "userfunction.h"
#include "flexible_connection.h"
#include "load.h"
#include "cuboid.h"
#include "cylinder.h"

using namespace AMVis;

System::System(const string &projectName) : MultiBodySystem(projectName) {

  // Gravitation
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

  BodyRigid *box1 = new BodyRigid("Stab1");
  addObject(box1);
  box1->setMass(m1);
  SymMat Theta(3);
  Theta(2,2) = 1./12.*m1*l1*l1;
  box1->setInertiaTensor(Theta);

  SqrMat E(3);
  E << DiagMat(3,INIT,1);
  Vec KrSP(3);
  KrSP(1) = a1;
  box1->addCoordinateSystem("PunktO",KrSP,E);

  box1->setTranslation( new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  box1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  box1->setFrameOfReference(getCoordinateSystem("I"));
  box1->setCoordinateSystemForKinematics(box1->getCoordinateSystem("C"));

  Vec q0(3);
  q0(0) = a1;
  q0(2) = -phi1;
  box1->setq0(q0);

  KrSP(1) = -b1;
  box1->addCoordinateSystem("PunktU",KrSP,E);

  BodyRigid *box2 = new BodyRigid("Stab2");
  addObject(box2);
  box2->setMass(m2);
  Theta(2,2) = 1./12.*m2*l2*l2;
  box2->setInertiaTensor(Theta);

  KrSP(1) = a2;
  box2->addCoordinateSystem("Punkt",KrSP,E);
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

  addCoordinateSystem("Os","[0;0;0.04]",E);
  box2->setFrameOfReference(getCoordinateSystem("Os"));
  box2->setCoordinateSystemForKinematics(box2->getCoordinateSystem("C"));

  ConnectionFlexible *ls = new ConnectionFlexible("Gelenk1");
  ls->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  //ls->setForceLaw(new LinearRegularizedConnection(SqrMat(2,EYE)*1e7, SqrMat(2,INIT,1)));
  ls->setForceLaw(new SimpleLinearRegularizedConnection(1e7, 1));
  //ls->setTranslationalStiffness(1e7);
  //ls->setTranslationalDamping(1);
  addLink(ls);
  ls->connect(getCoordinateSystem("I"),box1->getCoordinateSystem("PunktO"));

  ls = new ConnectionFlexible("Gelenk2");
  ls->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  //ls->setForceLaw(new LinearRegularizedConnection(SqrMat(2,EYE)*1e7, SqrMat(2,INIT,1)));
  ls->setForceLaw(new SimpleLinearRegularizedConnection(1e7, 1));
  //ls->setTranslationalStiffness(1e7);
  //ls->setTranslationalDamping(1);
  addLink(ls);
  ls->connect(box1->getCoordinateSystem("PunktU"),box2->getCoordinateSystem("Punkt"));

  ////////////////////// Visualisierung in AMVis ////////////////////////
  Cylinder * cylinder = new Cylinder(box1->getFullName(),1,false);
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l1);
  cylinder->setColor(0.5);
  box1->setAMVisBody(cylinder);
  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);


  cylinder = new Cylinder(box2->getFullName(),1,false);
  cylinder->setTopRadius(0.02);
  cylinder->setBaseRadius(0.02);
  cylinder->setHeight(l2);
  cylinder->setColor(0.1);
  box2->setAMVisBody(cylinder);
  cylinder -> setInitialTranslation(0,-0.5,0);
  cylinder -> setInitialRotation(1.5708,0,0);
}


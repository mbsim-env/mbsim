#include "rocking_rod.h"
#include "body_rigid_abs.h"
#include "contact_rigid.h"
#include "impact_rigid.h"
//#include "cuboid.h"
#include "objobject.h"

using namespace AMVis;

RockingRod::RockingRod(const string &projectName) : MultiBodySystem(projectName) {
  setProjectDirectory("plot");
  setPlotLevel(2);

  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  setGrav(grav);

  double l = 0.8;              // Laenge Balken
  double alpha = 3.0 * M_PI/180.; // Winkel der Anfangslage
  double deltax = 0.2;             // Abstand der Kontaktpunkte

  double   h =  0.02;
  double   d = 0.1;

  double mu  = 0.3;

  BodyRigidAbs *balken = new BodyRigidAbs("Rod");
  double m = 0.7;
  balken->setMass(m);
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  balken->setInertia(Theta);

  balken->setJT(Mat("[1,0; 0,1; 0,0]"));
  balken->setJR(Mat("[0; 0; 1]"));

  Vec WrOS0(3);
  WrOS0(1) = .5;
  WrOS0(2) = d/2;
  balken->setWrOK0(WrOS0);
  SqrMat AWK0(3);
  AWK0(0,0) = cos(alpha);
  AWK0(0,1) = -sin(alpha);
  AWK0(1,0) = sin(alpha);
  AWK0(1,1) = cos(alpha);
  AWK0(2,2) = 1;
  balken->setAWK0(AWK0);
  addObject(balken);

  Line *line = new Line("Line");
  Vec KrSC(3);
  KrSC(1) = -0.5*h;
  balken->addContour(line,KrSC);
  Vec n(3,INIT,0.0), b(3,INIT,0.0);
  b(2) =  1;
  n(1) = 1;
  line->setCn(n);
  line->setCb(b);

 // Cuboid *cubeoid = new Cuboid(balken->getFullName(),1,false);
 // cubeoid->setSize(l,h,d);
 // cubeoid->setColor(0);
 // balken->setAMVisBody(cubeoid);
  ObjObject *obj = new ObjObject(balken->getFullName(),1,false);
  obj->setObjFilename("objects/rod.obj");
  balken->setAMVisBody(obj);
  obj->setInitialRotation(M_PI/2,0,0);
  obj->setScaleFactor(0.1);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);

  Vec delta1(3); 
  delta1(0) = -deltax/2.;
  Point* point1 = new Point("Point1");
  addContour(point1,delta1);

  Vec delta2(3);
  delta2(0) = deltax/2.;
  Point* point2 = new Point("Point2");
  addContour(point2,delta2);

  ContactRigid *cr1S = new ImpactRigid("Contact1"); 
  cr1S->setFrictionDirections(1);
  cr1S->setFrictionCoefficient(mu);
  cr1S->connect(point1,balken->getContour("Line"));
  addLink(cr1S);

  ContactRigid *cr2S = new ImpactRigid("Contact2");
  cr2S->setFrictionDirections(1);
  cr2S->setFrictionCoefficient(mu);
  cr2S->connect(point2,balken->getContour("Line"));
  addLink(cr2S);

}


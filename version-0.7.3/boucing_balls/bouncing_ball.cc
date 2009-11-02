#include "bouncing_ball.h"
#include "body_rigid_abs.h"
#include "arrow.h"
//#include "sphere.h"

using namespace AMVis;

BouncingBall::BouncingBall(const string &projectName) : MultiBodySystem(projectName) {
  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  setGrav(grav);

  // Parameters
  double D = 0.1;              		
  double m = 100.e-3;
  SymMat Theta(3);
// Vollkugel
  Theta(0,0) = m*D*D* 2./5.;
// Kugelschale -- Fussball
//  Theta(0,0) = m*D*D* 2./3.;
  Theta(1,1) = Theta(0,0);
  Theta(2,2) = Theta(0,0);
//  double mu  = 0.28;

  // Body with relative kinematics
  BodyRigidAbs *ball = new BodyRigidAbs("Ball");
  addObject(ball);
  ball->setMass(m);
  ball->setInertia(Theta);
  
  Mat J(3,3,EYE);
  ball->setJT(J);
  ball->setJR(J);

  // Initial translation and rotation
  Vec WrOS0(3);
  Vec u0(6);
  u0(0) =  1.0;
  u0(1) = -5.0;
  u0(1) =  0.0;
  u0(3) =  20.0;
  u0(4) =  0.0;
  u0(5) =  0.0;
  WrOS0(1) = 0.15;
  ball->setWrOK0(WrOS0);
  ball->setu0(u0);

  // Contour definition
  Sphere *sp = new Sphere("Sphere");
  sp->setRadius(D/2.);
  ball->addContour(sp,Vec(3));
  Vec n(3,INIT,0.0), b(3,INIT,0.0);

  // Obstacles
  Plane* pl = new Plane("Table");
  pl->setCn(Vec("[0;-1;0]"));
  addContour(pl,Vec(3));

  // Contacts
//  ImpactRigid *ir = new ImpactRigid("Contact"); 
  ir = new ImpactRigid("Contact"); 
  ir->setFrictionDirections(2);
//  ir->setFrictionCoefficient(mu);
  ir->setNormalRestitutionCoefficient(0.9);
  ir->connect(pl,ball->getContour("Sphere"));
  ir->setPlotLevel(3);
  addLink(ir);

  // Visualisation with AMVis
  sp->createAMVisBody(true);

  AMVis::Arrow *aC = new AMVis::Arrow(ir->getFullName(),1,false);
  ir->addAMVisForceArrow(aC,150.,1);
}

void BouncingBall::init() {
  ir->setFrictionCoefficient(mu);
  MultiBodySystem::init();
}

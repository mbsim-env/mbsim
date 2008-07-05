#include "system.h"
#include "rigid_body.h"
#include "tree.h"
#include "cuboid.h"
#include "contour.h"
#include "flexible_connection.h"
#include "load.h"
#include "cube.h"

using namespace AMVis;

class Moment : public UserFunction {
  public:
    Vec operator()(double t) {
      Vec a(3);
      a(0) = 0.001*cos(t);
      a(1) = 0.0005*sin(t);
      //a(2) = 0.15*sin(t+M_PI/8);
      return a;
    }
};

System::System(const string &projectName) : MultiBodySystem(projectName) {
 // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);
 // Parameters
  double l = 0.1;              		
  double h =  0.1;
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  double alpha = 3.0 * M_PI/180.; 
  double deltax = 0.2;           
  double mu  = 0.3;

  CoordinateSystem* origin = new CoordinateSystem("O");
  Vec WrOK(3);
  WrOK(2) = d/2;;
  origin->setWrOP(WrOK);

  //TreeTest *tree = new TreeTest("Baum"); 
  //addObject(tree);
  
  BodyRigid* body = new BodyRigid("Body1");
  addObject(body);
  //tree->addObject(body);

  body->setParentCoordinateSystem(origin);
  body->setReferenceCoordinateSystem(body->getCoordinateSystem("COG"));
  body->setMass(m);
  body->setMomentOfInertia(Theta);
  body->setTranslation(new LinearTranslation("[1; 0; 0]"));
  //body->setfAPK(new RotationAxis(Vec("[0;0;1]")));
  //body->setfPJT(new ConstJacobian("[1, 0, 0; 0, 1, 0; 0, 0, 0]"));
  //body->setfPJT(new ConstJacobian("[1, 0; 0, 1; 0, 0]"));
  //body->setfPJR(new ConstJacobian("[0, 0, 0; 0, 0, 0; 0, 0, 1]"));

 // Initial translation and rotation
  SqrMat A(3);
  A(0,0) = 1;
  A(1,1) = 1;
  A(2,2) = 1;

  Vec KrSC(3);
  KrSC(0) = l/2;
  body->addCoordinateSystem("P",KrSC,A);


  // Visualisation with AMVis
  Cuboid *cubeoid = new Cuboid(body->getFullName(),1,false);
  cubeoid->setSize(l,h,d);
  cubeoid->setColor(0);

  body->setAMVisBody(cubeoid);
  BodyRigid *body2 = new BodyRigid("Body2");
  addObject(body2);

  body2->setParentCoordinateSystem(origin);
  body2->setReferenceCoordinateSystem(body2->getCoordinateSystem("COG"));
  body2->setMass(m);
  body2->setMomentOfInertia(Theta);
  body2->setTranslation(new LinearTranslation("[1; 0; 0]"));
  body2->setq0(Vec(1,INIT,2*l));
  body2->setu0(Vec(1,INIT,1));

  KrSC(0) = -l/2;
  body2->addCoordinateSystem("P",KrSC,A);

  ConnectionFlexible* cf = new ConnectionFlexible("Verbindung_U");
  addLink(cf);
  cf->connect(getCoordinateSystem("O"),body->getCoordinateSystem("P"));
  cf->setTranslationalStiffness(10);
  cf->setTranslationalDamping(0.1);
  cf->setForceDirection("[1;0;0]");

  cf = new ConnectionFlexible("Verbindung_K");
  addLink(cf);
  cf->connect(body->getCoordinateSystem("P"),body2->getCoordinateSystem("P"));
  cf->setTranslationalStiffness(10);
  cf->setTranslationalDamping(0.1);
  cf->setForceDirection("[1;0;0]");

  cubeoid = new Cuboid(body2->getFullName(),1,false);
  cubeoid->setSize(l,h,d);
  cubeoid->setColor(0.5);
  body2->setAMVisBody(cubeoid);

}


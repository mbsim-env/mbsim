#include "system.h"
#include "rigid_body.h"
#include "objobject.h"
#include "contour.h"
#include "contact.h"
#include "constitutive_laws.h"
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
  double l = 0.8;              		
  double h =  0.02;
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  double alpha = 3.0 * M_PI/180.; 
  double deltax = 0.2;           
  double mu  = 0.3;

  //CoSy* origin = new CoSy("O");
  //Vec WrOK(3);
  //WrOK(2) = d/2;;
  //origin->setWrOP(WrOK);

  RigidBody* body = new RigidBody("Rod");
  addObject(body);

  body->setFrameOfReference(getCoordinateSystem("I"));
  body->setCoordinateSystemForKinematics(body->getCoordinateSystem("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

 // Initial translation and rotation
  Vec q0(3);
  q0(1) = .5;
  q0(2) = alpha;
  body->setq0(q0);

  // Contour definition
  Line *line = new Line("Line");
  Vec KrSC(3);
  KrSC(1) = -0.5*h;
  body->addContour(line,KrSC,SqrMat(3,EYE));

  // Obstacles
  Vec delta1(3); 
  delta1(0) = -deltax/2.;
  Point* point1 = new Point("Point1");
  addContour(point1,delta1,SqrMat(3,EYE));

  Vec delta2(3);
  delta2(0) = deltax/2.;
  Point* point2 = new Point("Point2");
  addContour(point2,delta2,SqrMat(3,EYE));

  Contact *cr1S = new Contact("Contact1"); 
  cr1S->setContactForceLaw(new UnilateralConstraint);
  cr1S->setContactImpactLaw(new UnilateralNewtonImpact);
  cr1S->connect(point1,body->getContour("Line"));
  addLink(cr1S);

  Contact *cr2S = new Contact("Contact2");
  cr2S->setContactForceLaw(new UnilateralConstraint);
  cr2S->setContactImpactLaw(new UnilateralNewtonImpact);
  cr2S->setFrictionForceLaw(new PlanarCoulombFriction(mu));
  cr2S->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  cr2S->connect(point2,body->getContour("Line"));
  addLink(cr2S);

  // Visualisation with AMVis
  ObjObject *obj = new ObjObject(body->getFullName(),1,false);
  obj->setObjFilename("objects/rod.obj");
  body->setAMVisBody(obj);
  obj->setInitialRotation(M_PI/2,0,0);
  obj->setScaleFactor(0.1);
  obj->setCalculationOfNormals(3);
  obj->setVertexEPS(1e-5);
  obj-> setNormalEPS(1e-5);
  obj-> setAngleEPS(M_PI*2/9);
  // Cuboid *cubeoid = new Cuboid(body->getFullName(),1,false);
  // cubeoid->setSize(l,h,d);
  // cubeoid->setColor(0);
  // body->setAMVisBody(cubeoid);

}


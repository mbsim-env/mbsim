#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/cuboid.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cube.h"
#endif
#include "mbsim/environment.h"


using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  addFrame("Os",Vec(3),SqrMat(3,EYE));

  Plane *wall = new Plane("WandUnten");
  double phi = M_PI/2;
  SqrMat AWK(3);
  AWK(0,0) = cos(phi);
  AWK(0,1) = -sin(phi);
  AWK(1,1) = cos(phi);
  AWK(1,0) = sin(phi);
  AWK(2,2) = 1;
  addContour(wall,Vec(3),AWK);

  RigidBody* body = new RigidBody("Wuerfel");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));

  Mat J("[1,0,0;0,1,0;0,0,1]");
  body->setTranslation(new LinearTranslation(J));
  body->setRotation(new CardanAngles);
  double m = 0.1;
  double l,b,h;
  l = 0.1;
  b = 0.1;
  h = 0.1;
  Vec q0(6);
  q0(1) = 0.6;
  body->setInitialGeneralizedPosition(q0);
  body->setInitialGeneralizedVelocity("[2;0;1;3;2;-1]");
  body->setMass(m);
  SymMat Theta(3);
  double A=m/12.*(b*b+h*h);
  double  B=m/12.*(h*h+l*l);
  double  C=m/12.*(l*l+b*b);
  Theta(0,0)=A;
  Theta(1,1)=B;
  Theta(2,2)=C;
  body->setInertiaTensor(Theta);

  Cuboid *cuboid = new Cuboid("Wuerfel");
  cuboid->setLength(l);
  cuboid->setHeight(h);
  cuboid->setDepth(b);
  body->addContour(cuboid,Vec(3),SqrMat(3,EYE));

  Contact *cnf = new Contact("Kontakt_Wuerfel");
  cnf->setContactForceLaw(new UnilateralConstraint);
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.4));
  cnf->setFrictionForceLaw(new SpatialCoulombFriction(0.3));
  cnf->setFrictionImpactLaw(new SpatialCoulombImpact(0.3));
  //cnf->setPlotFeature(linkKinematics,disabled);
  //cnf->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e4,100));
  //cnf->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(0.3));
  cnf->connect(getContour("WandUnten"), body->getContour("Wuerfel"));
  addLink(cnf);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cube *obj = new OpenMBV::Cube;
  body->setOpenMBVRigidBody(obj);
  obj->setLength(l);
#endif
}


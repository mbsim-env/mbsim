#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/multi_contact.h"
#include "mbsim/contours/room.h"
#include "mbsim/contours/cuboid.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cube.h"
#include <openmbvcppinterface/sphere.h>
#endif
#include "mbsim/environment.h"


using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  RigidBody* roomBody = new RigidBody("RaumKoerper");
  addObject(roomBody);
  roomBody->setMass(1);
  roomBody->setInertiaTensor(SymMat3(EYE));
  roomBody->setFrameOfReference(getFrameI());
  Room* room = new Room("Raum");
  room->setLength(.3); //X
  room->setDepth(.2); //Y
  room->setHeight(0.1); //Z
#ifdef HAVE_OPENMBVCPPINTERFACE
  room->enableOpenMBV(true, 2);
#endif
  roomBody->addContour(room, Vec3(), SqrMat3(EYE));


  RigidBody* body = new RigidBody("Wuerfel");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));

  Mat J("[1,0,0;0,1,0;0,0,1]");
  body->setTranslation(new LinearTranslation(J));
  body->setRotation(new CardanAngles);
  double m = 0.1;
  double l,b,h;
  l = 0.01;
  b = 0.01;
  h = 0.01;
  Vec q0(6);
  body->setInitialGeneralizedPosition(q0);
  body->setInitialGeneralizedVelocity("[2;3;1;3;2;-1]");
  body->setMass(m);
  SymMat Theta(3);
  double A=m/12.*(b*b+h*h);
  double  B=m/12.*(h*h+l*l);
  double  C=m/12.*(l*l+b*b);
  Theta(0,0)=A;
  Theta(1,1)=B;
  Theta(2,2)=C;
  body->setInertiaTensor(Theta);

//  Point* point = new Point("Punkt");
//  body->addContour(point, Vec3(), SqrMat3(EYE));
//
//#ifdef HAVE_OPENMBVCPPINTERFACE
//  OpenMBV::Sphere *sphere = new OpenMBV::Sphere;
//  body->setOpenMBVRigidBody(sphere);
//  sphere->setRadius(0.01);
//#endif

//  MultiContact * crommpoint = new MultiContact("Raum_Punkt_Kontakt");
//  crommpoint->setContactForceLaw(new UnilateralConstraint);
//  crommpoint->setContactImpactLaw(new UnilateralNewtonImpact(1));
//  crommpoint->setFrictionForceLaw(new SpatialCoulombFriction(0.3));
//  crommpoint->setFrictionImpactLaw(new SpatialCoulombImpact(0.3));
//  crommpoint->connect(roomBody->getContour("Raum"), body->getContour("Punkt"));
//  addLink(crommpoint);

  Cuboid *cuboid = new Cuboid("Wuerfel");
  cuboid->setLength(l);
  cuboid->setHeight(h);
  cuboid->setDepth(b);
  body->addContour(cuboid,Vec(3),SqrMat(3,EYE));

  MultiContact *cnf = new MultiContact("Kontakt_Wuerfel");
  cnf->setContactForceLaw(new UnilateralConstraint);
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(1));
//  cnf->setFrictionForceLaw(new SpatialCoulombFriction(0.3));
//  cnf->setFrictionImpactLaw(new SpatialCoulombImpact(0.3));
  //cnf->setPlotFeature(linkKinematics,disabled);
  //cnf->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e4,100));
  //cnf->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(0.3));
  cnf->connect(roomBody->getContour("Raum"), body->getContour("Wuerfel"));
  addLink(cnf);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cube *obj = new OpenMBV::Cube;
  body->setOpenMBVRigidBody(obj);
  obj->setLength(l);
#endif
}


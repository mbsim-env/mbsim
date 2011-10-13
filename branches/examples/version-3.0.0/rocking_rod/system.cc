#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
//#include "openmbvcppinterface/ivbody.h"
#include "openmbvcppinterface/cuboid.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

extern bool rigidContacts;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.8;              		
  double h =  0.02;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);
  double alpha = 3.0 * M_PI/180.; 
  double deltax = 0.2;           
  double mu  = 0.3;

  RigidBody* body = new RigidBody("Rod");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new LinearTranslation("[1, 0; 0, 1; 0, 0]"));
  body->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  // Initial translation and rotation
  Vec q0(3);
  q0(1) = .5;
  q0(2) = alpha-M_PI/2;
  body->setInitialGeneralizedPosition(q0);

  // Contour definition
  Line *line = new Line("Line");
  Vec KrSC(3);
  KrSC(0) = 0.5*h;
  body->addContour(line,KrSC,SqrMat(3,EYE));
  line->setPlotFeature(globalVelocity,enabled);
  line->setPlotFeature(globalAcceleration,enabled);

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
  cr1S->connect(point1,body->getContour("Line"));
  addLink(cr1S);

  Contact *cr2S = new Contact("Contact2");
  cr2S->connect(point2,body->getContour("Line"));
  addLink(cr2S);

  if(rigidContacts) {
    cr1S->setContactForceLaw(new UnilateralConstraint);
    cr1S->setContactImpactLaw(new UnilateralNewtonImpact);
    cr1S->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    cr1S->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
    cr2S->setContactForceLaw(new UnilateralConstraint);
    cr2S->setContactImpactLaw(new UnilateralNewtonImpact);
    cr2S->setFrictionForceLaw(new PlanarCoulombFriction(mu));
    cr2S->setFrictionImpactLaw(new PlanarCoulombImpact(mu));
  }
  else {
    cr1S->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    cr1S->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
    cr2S->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    cr2S->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  getFrame("I")->setPlotFeature(globalPosition,enabled);
  getFrame("I")->setPlotFeature(globalVelocity,enabled);
  getFrame("I")->setPlotFeature(globalAcceleration,enabled);
  getContour("Point1")->setPlotFeature(globalPosition,enabled);
  getContour("Point1")->setPlotFeature(globalVelocity,enabled);
  getContour("Point1")->setPlotFeature(globalAcceleration,enabled);

#ifdef HAVE_OPENMBVCPPINTERFACE
  // Visualisation with OpenMBV
  //OpenMBV::IvBody *obj=new OpenMBV::IvBody;
  OpenMBV::Cuboid *obj=new OpenMBV::Cuboid;
  obj->setLength(l,h,l/5.);
  obj->setInitialRotation(0,0,M_PI/2);
  body->setOpenMBVRigidBody(obj);
#endif
}


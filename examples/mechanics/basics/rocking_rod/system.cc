#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/links/contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

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
  body->setTranslation(new LinearTranslation<VecV>("[1, 0; 0, 1; 0, 0]"));
  body->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));

  // Initial translation and rotation
  Vec q0(3);
  q0(1) = .5;
  q0(2) = alpha-M_PI/2;
  body->setInitialGeneralizedPosition(q0);

  // Contour definition
  Line *line = new Line("Line");
  Vec KrSC(3);
  KrSC(0) = 0.5*h;
  body->addFrame(new FixedRelativeFrame("P",KrSC,SqrMat(3,EYE)));
  line->setFrameOfReference(body->getFrame("P"));
  body->addContour(line);

  // Obstacles
  Vec delta1(3); 
  delta1(0) = -deltax/2.;
  Point* point1 = new Point("Point1");
  addFrame(new FixedRelativeFrame("P1",delta1,SqrMat(3,EYE)));
  point1->setFrameOfReference(getFrame("P1"));
  addContour(point1);

  Vec delta2(3);
  delta2(0) = deltax/2.;
  Point* point2 = new Point("Point2");
  addFrame(new FixedRelativeFrame("P2",delta2,SqrMat(3,EYE)));
  point2->setFrameOfReference(getFrame("P2"));
  addContour(point2);

  Contact *cr1S = new Contact("Contact1"); 
  cr1S->connect(point1,body->getContour("Line"));
  addLink(cr1S);

  Contact *cr2S = new Contact("Contact2");
  cr2S->connect(point2,body->getContour("Line"));
  addLink(cr2S);

  if(rigidContacts) {
    cr1S->setNormalForceLaw(new UnilateralConstraint);
    cr1S->setNormalImpactLaw(new UnilateralNewtonImpact);
    cr1S->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    cr1S->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
    cr2S->setNormalForceLaw(new UnilateralConstraint);
    cr2S->setNormalImpactLaw(new UnilateralNewtonImpact);
    cr2S->setTangentialForceLaw(new PlanarCoulombFriction(mu));
    cr2S->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  }
  else {
    cr1S->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    cr1S->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
    cr2S->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,1e4)));
    cr2S->setTangentialForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(mu)));
  }

#ifdef HAVE_OPENMBVCPPINTERFACE
  // Visualisation with OpenMBV
  //boost::shared_ptr<OpenMBV::IvBody> obj=OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  boost::shared_ptr<OpenMBV::Cuboid> obj=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  obj->setLength(l,h,l/5.);
  obj->setInitialRotation(0,0,M_PI/2);
  body->setOpenMBVRigidBody(obj);
#endif
}


#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/coilspring.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // acceleration of gravity
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // frames on environment 
  this->addFrame("L",Vec(3,INIT,1.),SqrMat(3,EYE));

  // bodies
  RigidBody *mass1 = new RigidBody("Mass1");
  RigidBody *mass2 = new RigidBody("Mass2");

  // attributes
  mass1->setMass(1.);
  mass1->setInertiaTensor(SymMat(3,EYE));
  mass1->setTranslation(new LinearTranslation("[0.58; 0.58; 0.58]"));
  mass1->setFrameOfReference(getFrame("L")); 
  mass1->setFrameForKinematics(mass1->getFrame("C"));
  mass2->setMass(2.);
  mass2->setInertiaTensor(SymMat(3,EYE));
  mass2->setTranslation(new LinearTranslation("[0.58; 0.58; 0.58]"));
  mass2->setFrameOfReference(mass1->getFrame("C")); 
  mass2->setFrameForKinematics(mass2->getFrame("C"));
  mass2->setInitialGeneralizedPosition(-nrm2(Vec(3,INIT,1)));

  // add body to dynamical system
  this->addObject(mass1);	
  this->addObject(mass2);	

  // spring
  SpringDamper *spring1 = new SpringDamper("Spring1");
  spring1->setForceFunction(new LinearSpringDamperForce(1,1,0));
  spring1->connect(mass1->getFrame("C"),this->getFrame("I"));
  SpringDamper *spring2 = new SpringDamper("Spring2");
  spring2->setForceFunction(new LinearSpringDamperForce(100,1,0));
  spring2->connect(mass2->getFrame("C"),this->getFrame("I"));

  // add spring to dynamical system
  this->addLink(spring1);
  this->addLink(spring2);

  // contact
  Sphere *sphere1 = new Sphere("Sphere1");
  sphere1->setRadius(0.2);
  sphere1->enableOpenMBV();
  mass1->addContour(sphere1,Vec(3,INIT,0.),SqrMat(3,EYE));
  Sphere *sphere2 = new Sphere("Sphere2");
  sphere2->setRadius(0.2);
  sphere2->enableOpenMBV();
  mass2->addContour(sphere2,Vec(3,INIT,0.),SqrMat(3,EYE));
  Contact *contact = new Contact("Contact");
  contact->connect(sphere1,sphere2);
  contact->setContactForceLaw(new UnilateralConstraint());
  contact->setContactImpactLaw(new UnilateralNewtonImpact(0.3));
  this->addLink(contact);

  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::CoilSpring* openMBVspring1=new OpenMBV::CoilSpring;
  openMBVspring1->setSpringRadius(0.1);
  openMBVspring1->setCrossSectionRadius(0.01);
  openMBVspring1->setNumberOfCoils(5);
  spring1->setOpenMBVSpring(openMBVspring1);
  
  OpenMBV::CoilSpring* openMBVspring2=new OpenMBV::CoilSpring;
  openMBVspring2->setSpringRadius(0.1);
  openMBVspring2->setCrossSectionRadius(0.01);
  openMBVspring2->setNumberOfCoils(5);
  spring2->setOpenMBVSpring(openMBVspring2);
#endif
}


#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/links/contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"

#include "openmbvcppinterface/coilspring.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // acceleration of gravity
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // frames on environment 
  this->addFrame(new FixedRelativeFrame("L",Vec(3,INIT,1.),SqrMat(3,EYE)));

  // bodies
  RigidBody *mass1 = new RigidBody("Mass1");
  RigidBody *mass2 = new RigidBody("Mass2");

  // attributes
  mass1->setMass(1.);
  mass1->setInertiaTensor(SymMat(3,EYE));
  mass1->setTranslation(new LinearTranslation<VecV>("[0.58; 0.58; 0.58]"));
  mass1->setFrameOfReference(getFrame("L")); 
  mass1->setFrameForKinematics(mass1->getFrame("C"));
  mass2->setMass(2.);
  mass2->setInertiaTensor(SymMat(3,EYE));
  mass2->setTranslation(new LinearTranslation<VecV>("[0.58; 0.58; 0.58]"));
  mass2->setFrameOfReference(mass1->getFrame("C")); 
  mass2->setFrameForKinematics(mass2->getFrame("C"));
  mass2->setGeneralizedInitialPosition(-nrm2(Vec(3,INIT,1)));

  // add body to dynamical system
  this->addObject(mass1);	
  this->addObject(mass2);	

  // spring
  SpringDamper *spring1 = new SpringDamper("Spring1");
  spring1->setForceFunction(new LinearSpringDamperForce(1,1));
  spring1->connect(mass1->getFrame("C"),this->getFrame("I"));
  SpringDamper *spring2 = new SpringDamper("Spring2");
  spring2->setForceFunction(new LinearSpringDamperForce(100,1));
  spring2->connect(mass2->getFrame("C"),this->getFrame("I"));

  // add spring to dynamical system
  this->addLink(spring1);
  this->addLink(spring2);

  // contact
  Sphere *sphere1 = new Sphere("Sphere1");
  sphere1->setRadius(0.2);
  sphere1->enableOpenMBV();
  mass1->addContour(sphere1);
  Sphere *sphere2 = new Sphere("Sphere2");
  sphere2->setRadius(0.2);
  sphere2->enableOpenMBV();
  mass2->addContour(sphere2);
  Contact *contact = new Contact("Contact");
  contact->connect(sphere1,sphere2);
  contact->setNormalForceLaw(new UnilateralConstraint());
  contact->setNormalImpactLaw(new UnilateralNewtonImpact(0.3));
  this->addLink(contact);

  // visualisation
  spring1->enableOpenMBV(_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);
  
  spring2->enableOpenMBV(_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
  setPlotFeatureRecursive(deflection, true);
}


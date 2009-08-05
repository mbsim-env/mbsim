#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cube.h"
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
  RigidBody *mass = new RigidBody("Mass");

  // attributes
  mass->setMass(1.);
  mass->setInertiaTensor(SymMat(3,EYE));
  mass->setTranslation(new LinearTranslation("[0.58; 0.58; 0.58]"));
  mass->setFrameOfReference(getFrame("L")); 
  mass->setFrameForKinematics(mass->getFrame("C"));

  // add body to dynamical system
  this->addObject(mass);	

  // spring
  SpringDamper *spring = new SpringDamper("Spring");
  spring->setForceFunction(new LinearSpringDamperForce(100,1,0));
  spring->connect(mass->getFrame("C"),this->getFrame("I"));

  // add spring to dynamical system
  this->addLink(spring);

  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cube *cuboid=new OpenMBV::Cube;
  cuboid->setLength(0.5);
  cuboid->setStaticColor(0.5);
  mass->setOpenMBVRigidBody(cuboid);

  OpenMBV::CoilSpring* openMBVspring=new OpenMBV::CoilSpring;
  openMBVspring->setSpringRadius(0.1);
  openMBVspring->setCrossSectionRadius(0.01);
  openMBVspring->setNumberOfCoils(5);
  spring->setOpenMBVSpring(openMBVspring);
#endif
}


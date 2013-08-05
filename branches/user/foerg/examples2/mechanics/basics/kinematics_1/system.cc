#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class MyPos : public Translation {
  public:
    int getqSize() const {return 0;}
    void updatePosition(const VecV &q, const double &t) {
      double om = 1;
      r(0) = cos(om*t);
      r(1) = sin(om*t);
    }; 
    void updateGuidingVelocity(const VecV &q, const double &t) {
      double om = 1;
      jh(0) = -sin(om*t)*om;
      jh(1) =  cos(om*t)*om;
    }
    void updateGyroscopicAcceleration(const VecV &u, const VecV &q, const double &t) {
      double om = 1;
      jb(0) = -cos(om*t)*om*om;
      jb(1) =  -sin(om*t)*om*om;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.3; 
#ifdef HAVE_OPENMBVCPPINTERFACE
  double h = 0.02;
  double d = 0.1;
#endif
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);

  RigidBody* body = new RigidBody("Rod");
  addObject(body);

  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  //body->setTranslation(new TranslationTeqI(new MyPos));
  body->setTranslation(new MyPos);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid *cuboid=new OpenMBV::Cuboid;
  cuboid->setLength(l,h,d);
  body->setOpenMBVRigidBody(cuboid);
#endif

  // Just to have somtething to integrate ;-)
  body = new RigidBody("Rod2");
  addObject(body);
  body->setFrameOfReference(getFrame("I"));
  body->setFrameForKinematics(body->getFrame("C"));
  body->setMass(m);
  body->setInertiaTensor(Theta);
  body->setTranslation(new LinearTranslation("[0; 1; 0]"));
}


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
    int getqSize() const {return 1;}
    void updatePosition(const VecV &q, const double &t) {
      r(0) = cos(q(0));
      r(1) = sin(q(0));
    }; 
    void updateJacobian(const VecV &q, const double &t) {
      J(0,0) = -sin(q(0));
      J(1,0) =  cos(q(0));
    }
    void updateGyroscopicAcceleration(const VecV &u, const VecV &q, const double &t) {
      jb(0) = -cos(q(0))*u(0)*u(0);
      jb(1) = -sin(q(0))*u(0)*u(0);
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.8; 
#ifdef HAVE_OPENMBVCPPINTERFACE
  double h = 0.02;  	 	 
  double d = 0.1;
#endif
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);

  RigidBody* body = new RigidBody("Rod");
  this->addObject(body);
  body->setFrameOfReference(this->getFrame("I"));
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


}


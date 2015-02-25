#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class MyPos : public MBSim::Function<Vec3(VecV)> {
  public:
    int getArgSize() const {return 1;}
    Vec3 operator()(const VecV &q) {
      Vec3 r;
      r(0) = cos(q(0));
      r(1) = sin(q(0));
      return r;
    }; 
    Mat3xV parDer(const VecV &q) {
      Mat3xV J(1);
      J(0,0) = -sin(q(0));
      J(1,0) =  cos(q(0));
      return J;
    }
    Mat3xV parDerDirDer(const VecV &qd, const VecV &q) {
      Mat3xV Jd(1);
      Jd(0,0) = -cos(q(0))*qd(0)*qd(0);
      Jd(1,0) = -sin(q(0))*qd(0)*qd(0);
      return Jd;
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
  boost::shared_ptr<OpenMBV::Cuboid> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  cuboid->setLength(l,h,d);
  body->setOpenMBVRigidBody(cuboid);
#endif


}


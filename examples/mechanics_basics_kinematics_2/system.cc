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
    Vec operator()(const Vec &q, double t) {
      Vec PrPK(3);
      PrPK(0) = cos(q(0));
      PrPK(1) = sin(q(0));
      return PrPK;
    }; 
};

class JacobianT : public Jacobian {
  public:
    int getuSize() const {return 1;} 
    Mat operator()(const Vec& q, double t) {
      Mat J(3,1);
      J(0,0) = -sin(q(0));
      J(1,0) =  cos(q(0));
      return J;
    }
};
class JacobianR : public Jacobian {
  public:
    Mat operator()(const Vec& q, double t) {
      Mat J(3,1);
      return J;
    }
};

class MyDerT : public Function3<Mat,Vec,Vec,double> {
  public:
    Mat operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
      Mat J(3,1);
      J(0,0) = -cos(q(0))*qd(0);
      J(1,0) = -sin(q(0))*qd(0);
      return J;
    }
};

class MyDerR : public Function3<Mat,Vec,Vec,double> {
  public:
    Mat operator()(const Vec &qd, const Vec& q, const double& t, const void*) {
      Mat J(3,1);
      return J;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.8; 
  double h = 0.02;  	 	 
  double d = 0.1;
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
  body->setTranslation(new MyPos);
  body->setJacobianOfTranslation(new JacobianT);
  body->setDerivativeOfJacobianOfTranslation(new MyDerT);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid *cuboid=new OpenMBV::Cuboid;
  cuboid->setLength(l,h,d);
  body->setOpenMBVRigidBody(cuboid);
#endif


}


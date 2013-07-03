#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

// class MyPos : public fmatvec::Function<Vec3(VecV, double)> {
//   public:
//     int getArg1Size() {return 0;}
//     Vec3 operator()(const VecV &q, const double &t) {
//       Vec3 PrPK;
//       double om = 1;
//       PrPK(0) = cos(om*t);
//       PrPK(1) = sin(om*t);
//       return PrPK;
//     }; 
//     Mat3xV parDer1(const VecV &q, const double &t) {
//       return Mat3xV();
//     }
//     Vec3 parDer2(const VecV &q, const double &t) {
//       Vec3 j;
//       double om = 1;
//       j(0) = -sin(om*t)*om;
//       j(1) =  cos(om*t)*om;
//       return j;
//     }
//     Mat3xV parDer1ParDer2(const VecV &q, const double &t) {
//       return Mat3xV();
//     }
//     Mat3xV parDer1DirDer1(const VecV &qd, const VecV &q, const double &t) {
//       return Mat3xV();
//     }
//     Vec3 parDer2ParDer2(const VecV &q, const double &t) {
//       Vec3 dj;
//       double om = 1;
//       dj(0) = -cos(om*t)*om*om;
//       dj(1) =  -sin(om*t)*om*om;
//       return dj;
//     }
//     Vec3 parDer2DirDer1(const VecV &qd, const VecV &q, const double &t) {
//       return Vec3();
//     }
// };

class MyPos : public TranslationTeqI {
  public:
    int getqSize() const {return 0;}
    void updatePosition(const VecV &q, const double &t) {
      double om = 1;
      r(0) = cos(om*t);
      r(1) = sin(om*t);
    }; 
    void updateJacobian(const VecV &q, const double &t) {
    }
    void updateGuidingVelocity(const VecV &q, const double &t) {
      double om = 1;
      j(0) = -sin(om*t)*om;
      j(1) =  cos(om*t)*om;
    }
    void updateDerivativeOfJacobian(const VecV &qd, const VecV &q, const double &t) {
    }
    void updateDerivativeOfGuidingVelocity(const VecV &qd, const VecV &q, const double &t) {
      double om = 1;
      jd(0) = -cos(om*t)*om*om;
      jd(1) =  -sin(om*t)*om*om;
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


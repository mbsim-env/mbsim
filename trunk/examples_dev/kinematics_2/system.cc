#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include "mbsim/userfunction.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public UserFunction {
  public:
    Vec operator()(double t) {
      Vec a(3);
      a(0) = 0.001*cos(t);
      a(1) = 0.0005*sin(t);
      return a;
    }
};

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

class MyDerT : public DerivativeOfJacobian {
  public:
    Mat operator()(const Vec &qd, const Vec& q, double t) {
      Mat J(3,1);
      J(0,0) = -cos(q(0))*qd(0);
      J(1,0) = -sin(q(0))*qd(0);
      return J;
    }
};

class MyDerR : public DerivativeOfJacobian {
  public:
    Mat operator()(const Vec &qd, const Vec& q, double t) {
      Mat J(3,1);
      return J;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);
  // Parameters
  double l = 0.8; 
  double h = 0.02;  	 	 
  double d = 0.1;
  double m = 0.7;
  SymMat Theta(3);
  Theta(1,1) = m*l*l/12.;
  Theta(2,2) = Theta(1,1);

  Tree *tree = new Tree("Baum"); 
  addDynamicSystem(tree,Vec(3),SqrMat(3,EYE));
  RigidBody* body = new RigidBody("Rod");

  tree->addObject(0, body);
  body->setFrameOfReference(tree->getFrame("I"));
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


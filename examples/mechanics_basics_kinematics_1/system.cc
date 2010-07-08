#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class MyRot : public Rotation {
  public:
    virtual SqrMat operator()(const Vec &q, double t) {
      SqrMat A(3);
      for(int i=0; i<3; i++)
        A(i,i) = 1;
      return A;
    }; 
};

class MyPos : public Translation {
  public:
    int getqSize() const {return 0;}
    virtual Vec operator()(const Vec &q, const double &t, const void * =NULL) {
      Vec PrPK(3);
      double om = 1;
      PrPK(0) = cos(om*t);
      PrPK(1) = sin(om*t);
      return PrPK;
    }; 
};

class jT : public Function1<Vec,double> {
  public:
    Vec operator()(const double& t, const void*) {
      Vec j(3);
      double om = 1;
      j(0) = -sin(om*t)*om;
      j(1) =  cos(om*t)*om;
      return j;
    }
};

class djT : public Function1<Vec,double> {
  public:
    Vec operator()(const double& t, const void*) {
      Vec dj(3);
      double om = 1;
      dj(0) = -cos(om*t)*om*om;
      dj(1) =  -sin(om*t)*om*om;
      return dj;
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
  body->setTranslation(new MyPos);
  body->setGuidingVelocityOfTranslation(new jT);
  body->setDerivativeOfGuidingVelocityOfTranslation(new djT);

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


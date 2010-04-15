#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/special_body.h"
#include "mbsim/constraint.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

class Moment : public Function1<fmatvec::Vec, double> {
  double M0;
  public:
    Moment(double M0_) : M0(M0_) {}
    fmatvec::Vec operator()(const double& tVal, const void * =NULL) {
      Vec M(1);
      M(0) = M0;
      return M;
    };
};

Gear::Gear(const string &projectName) : DynamicSystemSolver(projectName) {
  double R1 = 0.02;
  double m1 = 1;
  double J = 0.5*m1*R1*R1; 
  SymMat Theta(3);
  double l = 0.1;

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  Vec r(3);
  r(0) = l/2;
  addFrame("Q",r,BasicRotAKIy(M_PI/2));

  SpecialBody* shaft1 = new SpecialBody("Shaft1");
  addObject(shaft1);
  shaft1->setFrameOfReference(getFrame("I"));
  shaft1->setFrameForKinematics(shaft1->getFrame("C"));
  shaft1->setMass(m1);
  Theta(2,2) = J;
  shaft1->setInertiaTensor(Theta);
  shaft1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  shaft1->getFrame("C")->enableOpenMBV(0.3);
  //shaft1->setInitialGeneralizedVelocity("[1]");

  SpecialBody* shaft2 = new SpecialBody("Shaft2");
  addObject(shaft2);
  shaft2->setFrameOfReference(getFrame("Q"));
  shaft2->setFrameForKinematics(shaft2->getFrame("C"));
  shaft2->setMass(m1);
  Theta(2,2) = J;
  shaft2->setInertiaTensor(Theta);
  shaft2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  shaft2->getFrame("C")->enableOpenMBV(0.3);

  r(0) = 2*l;
  r(1) = l/5;
  addFrame("P",r,BasicRotAKIy(M_PI/2));
  SpecialBody* shaft3 = new SpecialBody("Shaft3");
  addObject(shaft3);
  shaft3->setFrameOfReference(getFrame("P"));
  shaft3->setFrameForKinematics(shaft3->getFrame("C"));
  shaft3->setMass(m1);
  Theta(2,2) = J;
  shaft3->setInertiaTensor(Theta);
  shaft3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  shaft3->getFrame("C")->enableOpenMBV(0.3);

  Constraint2 *constraint = new Constraint2("C1",shaft2);
  addObject(constraint);
  constraint->addDependency(shaft1,0.5);

  KineticExcitation* ke;
  ke = new KineticExcitation("MAn");
  addLink(ke);
  ke->connect(shaft1->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(1.1/100.));

  constraint = new Constraint2("C2",shaft3);
  addObject(constraint);
  constraint->addDependency(shaft2,0.5);

  ke = new KineticExcitation("MAbL");
  addLink(ke);
  ke->connect(shaft3->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(-4.0/100.));

  OpenMBV::Frustum *cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R1);
  cylinder->setBaseRadius(R1);
  cylinder->setHeight(l);
  cylinder->setStaticColor(0.1);
  shaft1->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R1);
  cylinder->setBaseRadius(R1);
  cylinder->setHeight(l);
  cylinder->setStaticColor(0.1);
  shaft2->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R1);
  cylinder->setBaseRadius(R1);
  cylinder->setHeight(l);
  cylinder->setStaticColor(0.1);
  shaft3->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);

}


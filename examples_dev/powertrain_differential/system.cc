#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsimPowertrain/shaft.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

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

Pendulum::Pendulum(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double m = 1;
  double m2 = 1.5;
  double m3 = 0.3;
  double m4 = 1;
  double m5 = m4;
  double l = 1;
  double l2 = l/10;
  double l3 = 0.3;
  double l4 = 1;
  double l5 = l4;
  double R = 0.2;
  double R2 = 2*R;
  double R3 = 0.1;
  double R4 = R3;
  double R5 = R4;
  double J = 0.5*m*R*R; 
  double J2 = 0.5*m2*R2*R2; 
  double J3 = 0.5*m3*R3*R3; 
  double J4 = 0.5*m4*R4*R4; 
  double J5 = 0.5*m5*R5*R5; 

  SymMat Theta(3);
  RigidBody* housing = new RigidBody("Housing");
  addObject(housing);

  housing->setFrameOfReference(getFrame("I"));
  housing->setFrameForKinematics(housing->getFrame("C"));

  housing->setMass(m);
  Theta(2,2) = J;
  housing->setInertiaTensor(Theta);
  housing->getFrame("C")->enableOpenMBV(0.3);

  Shaft* shaft1 = new Shaft("Shaft1");
  addObject(shaft1);
  //shaft1->setInitialGeneralizedVelocity(Vec(1,INIT,2));

  shaft1->setFrameOfReference(getFrame("I"));
  //shaft1->setFrameOfReference(housing->getFrame("C"));
  shaft1->setFrameForKinematics(shaft1->getFrame("C"));

  shaft1->setMass(m);
  Theta(2,2) = J;
  shaft1->setInertiaTensor(Theta);
  shaft1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  Vec r(3);
  r(2) = l/2;
  shaft1->addFrame("Q",r,SqrMat(3,EYE));
  shaft1->getFrame("Q")->enableOpenMBV(0.3);
  shaft1->getFrame("C")->enableOpenMBV(0.3);

  Shaft* shaft2 = new Shaft("Shaft2");
  addObject(shaft2);

  r(2) = l;
  r(1) = 1.5*R;
  //housing->addFrame("Q",r,SqrMat(3,EYE));
  //housing->getFrame("Q")->enableOpenMBV(0.3);
  //shaft2->setFrameOfReference(housing->getFrame("Q"));

  addFrame("Q",r,BasicRotAKIy(M_PI/2));
  getFrame("Q")->enableOpenMBV(0.3);
  shaft2->setFrameOfReference(getFrame("Q"));
  shaft2->setFrameForKinematics(shaft2->getFrame("C"));
  shaft2->getFrame("C")->enableOpenMBV(0.3);

  shaft2->setMass(m2);
  Theta(2,2) = J2;
  shaft2->setInertiaTensor(Theta);
  shaft2->addDependecy(shaft1,-2);

  Shaft* shaft3 = new Shaft("Shaft3");
  addObject(shaft3);
  //shaft3->setInitialGeneralizedVelocity(Vec(1,INIT,0.7));
  //shaft3->setInitialGeneralizedVelocity(Vec(1,INIT,0.0));

  r.init(0);
  r(2) = l2/2+R3*1.2;
  shaft2->addFrame("Q",r,BasicRotAKIy(M_PI/2));
  shaft2->getFrame("Q")->enableOpenMBV(0.3);
  shaft3->setFrameOfReference(shaft2->getFrame("Q"));
  shaft3->setFrameForKinematics(shaft3->getFrame("C"));
  shaft3->getFrame("C")->enableOpenMBV(0.3);

  shaft3->setMass(m3);
  Theta(2,2) = J3;
  shaft3->setInertiaTensor(Theta);
  shaft3->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  Shaft* shaft4 = new Shaft("Shaft4");
  addObject(shaft4);

  r.init(0);
  r(2) = -l4/2-0.2;
  addFrame("L",r,BasicRotAKIy(0),getFrame("Q"));
  getFrame("L")->enableOpenMBV(0.3);
  shaft4->setFrameOfReference(getFrame("L"));
  shaft4->setFrameForKinematics(shaft4->getFrame("C"));
  shaft4->getFrame("C")->enableOpenMBV(0.3);

  shaft4->setMass(m4);
  Theta(2,2) = J4;
  shaft4->setInertiaTensor(Theta);
  shaft4->addDependecy(shaft2,1);
  shaft4->addDependecy(shaft3,1);

  Shaft* shaft5 = new Shaft("Shaft5");
  addObject(shaft5);

  r.init(0);
  r(2) = +l5/2+0.2;
  addFrame("R",r,BasicRotAKIy(0),getFrame("Q"));
  getFrame("R")->enableOpenMBV(0.3);
  shaft5->setFrameOfReference(getFrame("R"));
  shaft5->setFrameForKinematics(shaft5->getFrame("C"));
  shaft5->getFrame("C")->enableOpenMBV(0.3);

  shaft5->setMass(m5);
  Theta(2,2) = J5;
  shaft5->setInertiaTensor(Theta);
  shaft5->addDependecy(shaft2,1);
  shaft5->addDependecy(shaft3,-1);
KineticExcitation* ke;
  ke = new KineticExcitation("MAn");
  addLink(ke);
  ke->connect(shaft1->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(1.2));
//
  ke = new KineticExcitation("MAbL");
  addLink(ke);
  ke->connect(shaft4->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(0.99));

  ke = new KineticExcitation("MAbR");
  addLink(ke);
  ke->connect(shaft5->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(1));


#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum *cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R);
  cylinder->setBaseRadius(R);
  cylinder->setHeight(l);
  cylinder->setStaticColor(0.1);
  shaft1->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R2);
  cylinder->setBaseRadius(R2);
  cylinder->setHeight(l2);
  cylinder->setStaticColor(0.4);
  shaft2->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l2/2);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R3);
  cylinder->setBaseRadius(R3);
  cylinder->setHeight(l3);
  cylinder->setStaticColor(0.5);
  shaft3->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l3/2);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R4);
  cylinder->setBaseRadius(R4);
  cylinder->setHeight(l4);
  cylinder->setStaticColor(0.5);
  shaft4->setOpenMBVRigidBody(cylinder);

  cylinder->setInitialTranslation(0,0,l5/2);
  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(R5);
  cylinder->setBaseRadius(R5);
  cylinder->setHeight(l5);
  cylinder->setStaticColor(0.5);
  shaft5->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l5/2);
#endif

}


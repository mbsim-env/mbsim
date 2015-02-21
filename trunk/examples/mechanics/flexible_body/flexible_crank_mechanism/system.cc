#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
#include "mbsimFlexibleBody/fixed_nodal_frame.h"
#include "mbsim/constraint.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public MBSim::Function<VecV(double)> {
  public:
    VecV operator()(const double& t) { 
      double t0 = 1;
      double t1 = 1.5;
      double M0 = 0*0.2;
      VecV M(1);
      if(t<t0)
        M(0) = t*M0/t0;
      else if(t<t1)
        M(0) = M0-(t-t0)/(t1-t0)*M0;
      else 
        M(0) = 0;
      return M;
    } 
};

CrankMechanism::CrankMechanism(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double E = 2.0e11;
  double nu = 0.3;
  double m1 = 0.36;
  double J1 = 0.002727; 
  double m2 = 0.151104;
  double J2 = 0.0045339259;
  double m3 = 0.075552; 
  double rho = 7870;
  double l1 = 0.15; 
  double l2 = 0.30;
  double d2 = 0.008;
  double h2 = 0.008;

  Vec  Kr(3);   
  SymMat Theta(3);

  Kr(0) = l1/2;

  RigidBody* body1 = new RigidBody("body1");
  addObject(body1);
  body1->addFrame(new FixedRelativeFrame("K", -Kr,SqrMat(3,EYE)));
  body1->addFrame(new FixedRelativeFrame("Q", Kr,SqrMat(3,EYE)));

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("K"));
  body1->setMass(m1);
  Theta(2,2)=J1;
  body1->setInertiaTensor(Theta);
  body1->setFrameForInertiaTensor(body1->getFrame("K"));
  body1->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));

  FlexibleBodyFFR* body2 = new FlexibleBodyFFR("body2");
  addObject(body2);
  Kr(0) = l2/2;
  Mat3xV T(4), P(4);
  T(0,3) = 1;
  P(2,0) = -M_PI/l2;
  P(2,1) = 2.*M_PI/l2;
  body2->addFrame(new FixedNodalFrame("Q", 2.*Kr, T, P));
#ifdef HAVE_OPENMBVCPPINTERFACE
  body2->getFrame("Q")->enableOpenMBV(0.3);
#endif
  body2->setFrameOfReference(body1->getFrame("Q"));
  body2->setc0(Kr);
  body2->setMass(m2);
  Theta(2,2)=J2;
  body2->setI0(Theta);

  Mat3xV C1(4);
  C1(1,0) = 2./M_PI;
  C1(0,2) = 2./3;
  C1(0,3) = 1./6;
  C1*=rho*d2*h2*l2;

  vector<vector<SqrMatV> > C3(3);
  for(int i=0; i<3; i++) {
    C3[i].resize(3);
    for(int j=0; j<3; j++)
      C3[i][j].resize(4);
  }
  C3[0][0](0,0) = 1./24*pow(h2*M_PI/l2,2);
  C3[0][0](1,1) = 1./6*pow(h2*M_PI/l2,2);
  C3[0][0](2,2) = 8./15;
  C3[0][0](2,3) = 1./15;
  C3[0][0](3,2) = C3[0][0](2,3);
  C3[0][0](3,3) = 2./15;
  C3[0][0]*=rho*d2*h2*l2;
  C3[1][1](0,0) = 1./2;
  C3[1][1](1,1) = 1./2;
  C3[1][1]*=rho*d2*h2*l2;
  C3[0][1](2,0) = 16./pow(M_PI,3);
  C3[0][1](3,0) = -8./pow(M_PI,3)+1./M_PI;
  C3[0][1](3,1) = -1./2/M_PI;
  C3[0][1]*=rho*d2*h2*l2;
  C3[1][0] = C3[0][1].T();
  C3[2][1] = C3[1][2].T();
  C3[2][0] = C3[0][2].T();

  vector<SqrMat3> C4;
  C4.push_back(SqrMat3(3));
  C4[0](1,0) = rho*d2*h2*pow(l2,2)*1./M_PI;
  C4.push_back(SqrMat3(3));
  C4[1](1,0) = rho*d2*h2*pow(l2,2)*-1./2/M_PI;
  C4.push_back(SqrMat3(3));
  C4[2](1,1) = -rho*d2*h2*pow(l2,2)*1./3;
  C4[2](2,2) = C4[2](1,1);
  C4.push_back(SqrMat3(3));
  C4[3](1,1) = -rho*d2*h2*pow(l2,2)*1./6;
  C4[3](2,2) = C4[3](1,1);

  SymMatV Ke(4);
  Ke(0,0) = 1./24*pow(M_PI,4)*pow(h2/l2,2);
  Ke(1,1) = 2./3*pow(M_PI,4)*pow(h2/l2,2);
  Ke(2,2) = 16./3;
  Ke(2,3) = -8./3;
  Ke(3,3) = 7./3;
  Ke*=E/(1-pow(nu,2))*d2*h2/l2;

  body2->setC1(C1);
  body2->setC3(C3);
  body2->setC4(C4);
  body2->setKe(Ke);

  body2->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  body2->getFrame("Q")->setPlotFeature(globalPosition,enabled);
  
  static_cast<FixedNodalFrame*>(body2->getFrame("Q"))->setsigmahel("[0, 0, -2.9304e+12, 2.1978e+12; 0, 0, -8.79121e+11, 6.59341e+11; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0]");

  RigidBody* body3 = new RigidBody("body3");
  addObject(body3);
  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m3);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongXAxis<VecV>);
  body3->setInitialGeneralizedPosition(VecV(1,INIT,l1+l2));

  body1->setPlotFeature(stateDerivative,enabled);
  body2->setPlotFeature(stateDerivative,enabled);
  body3->setPlotFeature(stateDerivative,enabled);

  Joint* constraint = new Joint("C");
  constraint->connect(body3->getFrame("C"),body2->getFrame("Q"));
  addLink(constraint);
  constraint->setForceDirection("[1,0;0,1;0,0]");
  constraint->setForceLaw(new BilateralConstraint);

  //  KineticExcitation *load = new KineticExcitation("Motor");
  //  addLink(load);
  //  load->setMomentDirection("[0;0;1]");
  //  load->setMomentFunction(new Moment);
  //  load->connect(body1->getFrame("C"));

#if HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid* dummy = new OpenMBV::Cuboid;
  dummy->setLength(l1,h2,d2);
  dummy->setDiffuseColor(90./360.,1,1);
  body1->setOpenMBVRigidBody(dummy);
  dummy = new OpenMBV::Cuboid;
  dummy->setLength(l2,h2,d2);
  dummy->setDiffuseColor(180./360.,1,1);
  dummy->setInitialTranslation(l2/2,0,0);
  body2->setOpenMBVRigidBody(dummy);
  dummy = new OpenMBV::Cuboid;
  dummy->setLength(l2/2,h2,d2);
  dummy->setDiffuseColor(240./360.,1,1);
  body3->setOpenMBVRigidBody(dummy);
#endif


}
//

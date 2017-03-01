#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"

#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"

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
  body2->addFrame(new NodeFrame("Q",0));
  body2->getFrame("Q")->enableOpenMBV(0.3);
  body2->setFrameOfReference(body1->getFrame("Q"));

  vector<Vec3> rNod(1);
  rNod[0] = 2.*Kr;

  vector<Mat3xV> TNod(1), PNod(1);
  TNod[0] = T;
  PNod[0] = P;

  vector<Matrix<General, Fixed<6>, Var, double> > sigmahel(1);
  sigmahel[0] = Matrix<General, Fixed<6>, Var, double>("[0, 0, -2.9304e+12, 2.1978e+12; 0, 0, -8.79121e+11, 6.59341e+11; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0]");

  body2->setNodalRelativePosition(rNod);
  body2->setNodalShapeMatrixOfTranslation(TNod);
  body2->setNodalShapeMatrixOfRotation(PNod);
  body2->setNodalStressMatrix(sigmahel);

  SymMat rrdm(3);
  rrdm(0,0) = J2;

  Mat3xV Pdm(4);
  Pdm(1,0) = 2./M_PI;
  Pdm(0,2) = 2./3;
  Pdm(0,3) = 1./6;
  Pdm*=rho*d2*h2*l2;

  vector<vector<SqrMatV> > PPdm(3,vector<SqrMatV>(3));
  vector<Mat3xV> rPdm(3);
  for(int i=0; i<3; i++) {
    rPdm[i].resize(4);
    for(int j=0; j<3; j++) {
      PPdm[i][j].resize(4);
    }
  }
  PPdm[0][0](0,0) = 1./24*pow(h2*M_PI/l2,2);
  PPdm[0][0](1,1) = 1./6*pow(h2*M_PI/l2,2);
  PPdm[0][0](2,2) = 8./15;
  PPdm[0][0](2,3) = 1./15;
  PPdm[0][0](3,2) = PPdm[0][0](2,3);
  PPdm[0][0](3,3) = 2./15;
  PPdm[0][0]*=rho*d2*h2*l2;
  PPdm[1][1](0,0) = 1./2;
  PPdm[1][1](1,1) = 1./2;
  PPdm[1][1]*=rho*d2*h2*l2;
  PPdm[0][1](2,0) = 16./pow(M_PI,3);
  PPdm[0][1](3,0) = -8./pow(M_PI,3)+1./M_PI;
  PPdm[0][1](3,1) = -1./2/M_PI;
  PPdm[0][1]*=rho*d2*h2*l2;
  PPdm[1][0] = PPdm[0][1].T();
  PPdm[2][1] = PPdm[1][2].T();
  PPdm[2][0] = PPdm[0][2].T();

  rPdm[0](0,2) = rho*d2*h2*pow(l2,2)*1./3;
  rPdm[0](0,3) = rho*d2*h2*pow(l2,2)*1./6;
  rPdm[0](1,0) = rho*d2*h2*pow(l2,2)*1./M_PI;
  rPdm[0](1,1) = rho*d2*h2*pow(l2,2)*-1./2/M_PI;

  SymMatV Ke(4);
  Ke(0,0) = 1./24*pow(M_PI,4)*pow(h2/l2,2);
  Ke(1,1) = 2./3*pow(M_PI,4)*pow(h2/l2,2);
  Ke(2,2) = 16./3;
  Ke(2,3) = -8./3;
  Ke(3,3) = 7./3;
  Ke*=E/(1-pow(nu,2))*d2*h2/l2;

  body2->setMass(m2);
  body2->setPositionIntegral(m2*Kr);
  body2->setPositionPositionIntegral(rrdm);
  body2->setShapeFunctionIntegral(Pdm);
  body2->setShapeFunctionShapeFunctionIntegral(PPdm);
  body2->setPositionShapeFunctionIntegral(rPdm);
  body2->setStiffnessMatrix(Ke);

  body2->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  body2->getFrame("Q")->setPlotFeature(globalPosition,enabled);
  
  RigidBody* body3 = new RigidBody("body3");
  addObject(body3);
  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m3);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongXAxis<VecV>);
  body3->setGeneralizedInitialPosition(VecV(1,INIT,l1+l2));

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

  std::shared_ptr<OpenMBV::Cuboid> dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l1,h2,d2);
  dummy->setDiffuseColor(90./360.,1,1);
  body1->setOpenMBVRigidBody(dummy);
  dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l2/2,h2,d2);
  dummy->setDiffuseColor(240./360.,1,1);
  body3->setOpenMBVRigidBody(dummy);

}

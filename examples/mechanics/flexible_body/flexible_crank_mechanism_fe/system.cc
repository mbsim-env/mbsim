#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsimFlexibleBody/frames/fixed_nodal_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/links/kinetic_excitation.h"
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

CrankMechanism::CrankMechanism(const string &name, int n) : DynamicSystemSolver(name) {

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
  Mat3xV T(2*n), P(2*n);
  body2->addFrame(new FixedNodalFrame("Q", 2.*Kr, T, P));
#ifdef HAVE_OPENMBVCPPINTERFACE
  body2->getFrame("Q")->enableOpenMBV(0.3);
#endif
  body2->setFrameOfReference(body1->getFrame("Q"));
  body2->setc0(Kr);
  body2->setMass(m2);
  Theta(2,2)=J2;
  body2->setI0(Theta);

  double D = l2/n;
  Mat3xV C1(2*n);
  Mat3xV C1g(2*n+2);
  vector<vector<SqrMatV> > C3e(3);
  vector<vector<SqrMatV> > C3g(3);
  vector<vector<SqrMatV> > C3(3);
  for(int i=0; i<3; i++) {
    C3e[i].resize(3);
    C3g[i].resize(3);
    C3[i].resize(3);
    for(int j=0; j<3; j++) {
      C3e[i][j].resize(4);
      C3g[i][j].resize(2*n+2);
      C3[i][j].resize(2*n);
    }
  }
  vector<SqrMat3> C4e(4);
  vector<SqrMat3> C4g(2*n+2);
  vector<SqrMat3> C4(2*n);
  SymMatV Kee(4);
  SqrMatV Keg(2*n+2);
  SymMatV Ke(2*n);
  for(int i=0; i<n; i++) {
    int i1=i*2;
    int i2=i1+2+1;

    Mat3xV C1e(4);
    C1e(1,0) = 1./2;
    C1e(1,1) = 1./12*D;
    C1e(1,2) = 1./2; 
    C1e(1,3) = -1./12*D;
    C1e*=rho*d2*h2*D;
    C1g.add(Index(0,2),Index(i1,i2),C1e);

    C3e[0][0](0,0) = 6./5/D;
    C3e[0][0](0,1) = 1./10;
    C3e[0][0](0,2) = -6./5/D;
    C3e[0][0](0,3) = 1./10;
    C3e[0][0](1,1) = 2*D/15;
    C3e[0][0](1,2) = -1./10;
    C3e[0][0](1,3) = -D/30;
    C3e[0][0](2,2) = 6./5/D;
    C3e[0][0](2,3) = -1./10;
    C3e[0][0](3,3) = 2*D/15;
    C3e[1][1](0,0) = 13./35*D;
    C3e[1][1](0,1) = 11./210*pow(D,2);
    C3e[1][1](0,2) = 9./70*D;
    C3e[1][1](0,3) = -13./420*pow(D,2);
    C3e[1][1](1,1) = pow(D,3)/105;
    C3e[1][1](1,2) = 13./420*pow(D,2);
    C3e[1][1](1,3) = -pow(D,3)/140;
    C3e[1][1](2,2) = 13./35*D;
    C3e[1][1](2,3) = -11./210*pow(D,2);
    C3e[1][1](3,3) = pow(D,3)/105;
    for(int k=0; k<4; k++) {
      for(int j=0; j<k; j++) {
	C3e[0][0](k,j) = C3e[0][0](j,k);
	C3e[1][1](k,j) = C3e[1][1](j,k);
      }
    }
    C3e[0][0] *= 1./12*rho*d2*pow(h2,3);
    C3e[1][1] *= rho*d2*h2;
    for(int j=0; j<3; j++) 
      for(int k=0; k<3; k++)
	C3g[j][k].add(Index(i1,i2),Index(i1,i2),C3e[j][k]);

    C4e[0](0,1) = rho*1./12*d2*pow(h2,3);
    C4e[0](1,0) = rho*d2*h2*pow(D,2)*((i+1)/2.-7./20);
    C4e[1](1,0) = rho*d2*h2*pow(D,2)*((i+1)*D/12.-D/20);
    C4e[2](0,1) = rho*-1./12*d2*pow(h2,3);
    C4e[2](1,0) = rho*d2*h2*pow(D,2)*((i+1)/2.-3./20);
    C4e[3](1,0) = rho*d2*h2*pow(D,2)*(-(i+1)*D/12+D/30);
    int k=0;
    for(int j=i1; j<=i2; j++)
      C4g[j] += C4e[k++];

    Kee(0,0) = 12./pow(D,3);
    Kee(0,1) = 6./pow(D,2);
    Kee(0,2) = -12./pow(D,3);
    Kee(0,3) = 6./pow(D,2);
    Kee(1,1) = 4./D;
    Kee(1,2) = -6./pow(D,2);
    Kee(1,3) = 2./D;
    Kee(2,2) = 12./pow(D,3);
    Kee(2,3) = -6./pow(D,2);
    Kee(3,3) = 4./D;
    Kee*=E/(12*(1-pow(nu,2)))*d2*pow(h2,3);
    Keg.add(Index(i1,i2),Index(i1,i2),Kee);
  }

  int j=0;
  for(int i=1; i<2*n; i++)
    C1.set(j++,C1g.col(i));
  C1.set(j++,C1g.col(2*n+1));

  for(int j=0; j<3; j++) {
    for(int k=0; k<3; k++) {
      int h=0;
      for(int ii=0; ii<2*n+2; ii++) {
	int r=0;
	if(ii!=0 and ii!=2*n) {
	  for(int jj=0; jj<2*n+2; jj++)
	    if(jj!=0 and jj!=2*n) {
	      C3[j][k](h,r) = C3g[j][k](ii,jj);
	      r++;
	    }
	  h++;
	}
      }
    }
  }

  int k=0;
  for(int i=0; i<2*n+2; i++)
    if(i!=0 and i!=2*n) 
      C4[k++] = C4g[i];

  int h=0;
  for(int ii=0; ii<2*n+2; ii++) {
    int r=0;
    if(ii!=0 and ii!=2*n) {
      for(int jj=0; jj<2*n+2; jj++)
	if(jj!=0 and jj!=2*n) {
	  Ke(h,r) = Keg(ii,jj);
	  r++;
	}
      h++;
    }
  }

  body2->setC1(C1);
  body2->setC3(C3);
  body2->setC4(C4);
  body2->setKe(Ke);

  body2->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  body2->getFrame("Q")->setPlotFeature(globalPosition,enabled);
  
//  static_cast<FixedNodalFrame*>(body2->getFrame("Q"))->setsigmahel("[0, 0, -2.9304e+12, 2.1978e+12; 0, 0, -8.79121e+11, 6.59341e+11; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0]");

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
  boost::shared_ptr<OpenMBV::Cuboid> dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l1,h2,d2);
  dummy->setDiffuseColor(90./360.,1,1);
  body1->setOpenMBVRigidBody(dummy);
  dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l2,h2,d2);
  dummy->setDiffuseColor(180./360.,1,1);
  dummy->setInitialTranslation(l2/2,0,0);
  body2->setOpenMBVRigidBody(dummy);
  dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l2/2,h2,d2);
  dummy->setDiffuseColor(240./360.,1,1);
  body3->setOpenMBVRigidBody(dummy);
#endif


}
//

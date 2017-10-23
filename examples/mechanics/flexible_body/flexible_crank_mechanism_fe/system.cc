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
  body2->addFrame(new NodeFrame("Q",0));
  body2->getFrame("Q")->enableOpenMBV(0.3);
  body2->setFrameOfReference(body1->getFrame("Q"));

  vector<Vec3> rNod(1);
  rNod[0] = 2.*Kr;

  vector<Mat3xV> TNod(1), PNod(1);
  TNod[0] = T;
  PNod[0] = P;

  body2->setNodalRelativePosition(rNod);
  body2->setNodalShapeMatrixOfTranslation(TNod);
  body2->setNodalShapeMatrixOfRotation(PNod);

  SymMat rrdm(3);
  rrdm(0,0) = J2;

  double D = l2/n;
  Mat3xV Pdm(2*n);
  Mat3xV Pdmg(2*n+2);
  vector<vector<SqrMatV> > PPdme(3,vector<SqrMatV>(3));
  vector<vector<SqrMatV> > PPdmg(3,vector<SqrMatV>(3));
  vector<vector<SqrMatV> > PPdm(3,vector<SqrMatV>(3));
  vector<Mat3xV> rPdme(3);
  vector<Mat3xV> rPdmg(3);
  vector<Mat3xV> rPdm(3);
  for(int i=0; i<3; i++) {
    rPdme[i].resize(4);
    rPdmg[i].resize(2*n+2);
    rPdm[i].resize(2*n);
    for(int j=0; j<3; j++) {
      PPdme[i][j].resize(4);
      PPdmg[i][j].resize(2*n+2);
      PPdm[i][j].resize(2*n);
    }
  }
  SymMatV Kee(4);
  SqrMatV Keg(2*n+2);
  SymMatV Ke(2*n);
  for(int i=0; i<n; i++) {
    int i1=i*2;
    int i2=i1+2+1;

    Mat3xV Pdme(4);
    Pdme(1,0) = 1./2;
    Pdme(1,1) = 1./12*D;
    Pdme(1,2) = 1./2;
    Pdme(1,3) = -1./12*D;
    Pdme*=rho*d2*h2*D;
    Pdmg.add(RangeV(0,2),RangeV(i1,i2),Pdme);

    PPdme[0][0](0,0) = 6./5/D;
    PPdme[0][0](0,1) = 1./10;
    PPdme[0][0](0,2) = -6./5/D;
    PPdme[0][0](0,3) = 1./10;
    PPdme[0][0](1,1) = 2*D/15;
    PPdme[0][0](1,2) = -1./10;
    PPdme[0][0](1,3) = -D/30;
    PPdme[0][0](2,2) = 6./5/D;
    PPdme[0][0](2,3) = -1./10;
    PPdme[0][0](3,3) = 2*D/15;
    PPdme[1][1](0,0) = 13./35*D;
    PPdme[1][1](0,1) = 11./210*pow(D,2);
    PPdme[1][1](0,2) = 9./70*D;
    PPdme[1][1](0,3) = -13./420*pow(D,2);
    PPdme[1][1](1,1) = pow(D,3)/105;
    PPdme[1][1](1,2) = 13./420*pow(D,2);
    PPdme[1][1](1,3) = -pow(D,3)/140;
    PPdme[1][1](2,2) = 13./35*D;
    PPdme[1][1](2,3) = -11./210*pow(D,2);
    PPdme[1][1](3,3) = pow(D,3)/105;
    for(int k=0; k<4; k++) {
      for(int j=0; j<k; j++) {
	PPdme[0][0](k,j) = PPdme[0][0](j,k);
	PPdme[1][1](k,j) = PPdme[1][1](j,k);
      }
    }
    PPdme[0][0] *= 1./12*rho*d2*pow(h2,3);
    PPdme[1][1] *= rho*d2*h2;

    rPdme[0](1,0) = rho*d2*h2*pow(D,2)*((i+1)/2.-7./20);
    rPdme[0](1,1) = rho*d2*h2*pow(D,2)*((i+1)*D/12.-D/20);
    rPdme[0](1,2) = rho*d2*h2*pow(D,2)*((i+1)/2.-3./20);
    rPdme[0](1,3) = rho*d2*h2*pow(D,2)*(-(i+1)*D/12+D/30);
    rPdme[1](0,0) = rho*1./12*d2*pow(h2,3);
    rPdme[1](0,2) = rho*-1./12*d2*pow(h2,3);

    for(int j=0; j<3; j++) {
      rPdmg[j].add(RangeV(0,2),RangeV(i1,i2),rPdme[j]);
      for(int k=0; k<3; k++) {
	PPdmg[j][k].add(RangeV(i1,i2),RangeV(i1,i2),PPdme[j][k]);
      }
    }

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
    Keg.add(RangeV(i1,i2),RangeV(i1,i2),Kee);
  }

  int j=0;
  for(int i=1; i<2*n; i++)
    Pdm.set(j++,Pdmg.col(i));
  Pdm.set(j++,Pdmg.col(2*n+1));

  for(int j=0; j<3; j++) {
    for(int k=0; k<3; k++) {
      int h=0;
      for(int ii=0; ii<2*n+2; ii++) {
	int r=0;
	if(ii!=0 and ii!=2*n) {
	  for(int jj=0; jj<2*n+2; jj++) {
	    if(jj!=0 and jj!=2*n) {
	      PPdm[j][k](h,r) = PPdmg[j][k](ii,jj);
	      r++;
	    }
          }
          rPdm[j](k,h) = rPdmg[j](k,ii);
	  h++;
	}
      }
    }
  }

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

  body2->setMass(m2);
  body2->setPositionIntegral(m2*Kr);
  body2->setPositionPositionIntegral(rrdm);
  body2->setShapeFunctionIntegral(Pdm);
  body2->setShapeFunctionShapeFunctionIntegral(PPdm);
  body2->setPositionShapeFunctionIntegral(rPdm);
  body2->setStiffnessMatrix(Ke);

  body2->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  body2->getFrame("Q")->setPlotFeature(position,enabled);
  body2->getFrame("Q")->setPlotFeature(angle,enabled);

  RigidBody* body3 = new RigidBody("body3");
  addObject(body3);
  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m3);
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new TranslationAlongXAxis<VecV>);
  body3->setGeneralizedInitialPosition(VecV(1,INIT,l1+l2));

  body1->setPlotFeature(derivativeOfGeneralizedPosition,enabled);
  body1->setPlotFeature(generalizedAcceleration,enabled);
  body2->setPlotFeature(derivativeOfGeneralizedPosition,enabled);
  body2->setPlotFeature(generalizedAcceleration,enabled);
  body3->setPlotFeature(derivativeOfGeneralizedPosition,enabled);
  body3->setPlotFeature(generalizedAcceleration,enabled);

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

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}

#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
#include "mbsimFlexibleBody/frames/fixed_nodal_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/nested_function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Angle : public MBSim::Function<double(double)> {
  private:
    double T0, Om;
  public:
    Angle() : T0(30), Om(10) { }
    double operator()(const double& t) { 
      if(t<=T0)
        return Om/T0*(0.5*pow(t,2)+pow(T0/2/M_PI,2)*cos(2*M_PI*t/T0));
      else 
        return Om*t;
    } 
    double parDer(const double& t) { 
      if(t<=T0)
        return Om/T0*(t-T0/2/M_PI*sin(2*M_PI*t/T0));
      else 
        return Om;
    }
    double parDerParDer(const double& t) { 
      if(t<=T0)
        return Om/T0*(1-cos(2*M_PI*t/T0));
      else 
        return Om;
    }
};

CrankMechanism::CrankMechanism(const string &name, int stiffening) : DynamicSystemSolver(name) {

  Vec grav(3);
//  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double E = 1.17e11;
  double nu = 0*0.3;
  double rho = 1.2167e4;
  double l = 20.3;
  double A = 4e-4;
  double I = 9.83e-5;
  double h =  1.7173;
  double d =  2.3293e-04;

  double m = rho*l*h*d;
  double J = 1./3*m*pow(l,2);

  Vec  Kr(3);   
  SymMat Theta(3);

  FlexibleBodyFFR* body = new FlexibleBodyFFR("body");
  addObject(body);
  Kr(0) = l/2;
  Mat3xV T(2), P(2);
  T(0,0) = 1;
  T(1,1) = 1;
  P(2,1) = 0.067808;
  body->addFrame(new FixedNodalFrame("Q", 2.*Kr, T, P));
#ifdef HAVE_OPENMBVCPPINTERFACE
  body->getFrame("K")->enableOpenMBV(0.3);
  body->getFrame("Q")->enableOpenMBV(0.3);
#endif
  body->setc0(Kr);
  body->setMass(m);
  Theta(2,2)=J;
  body->setI0(Theta);

  Mat3xV C1(2);
  C1(0,0) = rho*0.00516935;
  C1(1,1) = rho*0.00317895;

  vector<vector<SqrMatV> > C3(3);
  for(int i=0; i<3; i++) {
    C3[i].resize(3);
    for(int j=0; j<3; j++)
      C3[i][j].resize(2);
  }
  C3[0][0](0,0) = rho*0.00406;
  C3[0][0](1,1) = 0;

  C3[1][1](0,0) = 0;
  C3[1][1](1,1) = rho*0.00203;

  C3[0][1](0,1) = rho*0.00275212;

  C3[1][0] = C3[0][1].T();
  C3[2][1] = C3[1][2].T();
  C3[2][0] = C3[0][2].T();

  vector<SqrMat3> C4;
  C4.push_back(SqrMat3(3));
  C4[0](1,1) = -rho*0.0668055;
  C4[0](2,2) = -rho*0.0668055;
  C4.push_back(SqrMat3(3));
  C4[1](1,0) = rho*0.0468815;

  SymMatV Ke(2);
  Ke(0,0) = 2.8442e+06;
  Ke(1,1) = 4249.06;

  body->setC1(C1);
  body->setC3(C3);
  body->setC4(C4);
  body->setKe(Ke);

  if(stiffening==1) {
    std::vector<SqrMatV> K0om(6);
    for(unsigned int i=0; i<K0om.size(); i++)
      K0om[i].resize(2);
    K0om[2](1,1) = rho*0.00242247;

    body->setK0om(K0om);
  }
  else if(stiffening==2) {
    vector<SqrMatV> C7(2);
    vector<vector<SqrMatV> > C8(2);
    C8[0].resize(2);
    C8[1].resize(2);

    C7[0].resize() = SqrMatV("[280216, 0; 0, 457.203]");
    C7[1].resize() = SqrMatV("[0, 49630; 98802.9, 0]");

    C8[0][0].resize() = SqrMatV("[6386.14, 0; 0, 11.0674]");
    C8[0][1].resize() = SqrMatV("[0, 1216.23; 1216.23, 0]");
    C8[1][0].resize() = SqrMatV("[0, 1216.23; 1216.23, 0]");
    C8[1][1].resize() = SqrMatV("[11.0674, 0; 0, 5039.25]");

    body->setC7(C7);
    body->setC8(C8);
  }

  body->setRotation(new NestedFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>("[0;0;1]"), new Angle));
  body->getFrame("K")->setPlotFeature(globalPosition,enabled);
  body->getFrame("K")->setPlotFeature(globalVelocity,enabled);
  body->getFrame("K")->setPlotFeature(globalAcceleration,enabled);

#if HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Cuboid> dummy = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  dummy->setLength(l,h,d);
  dummy->setDiffuseColor(180./360.,1,1);
  dummy->setInitialTranslation(l/2,0,0);
  body->setOpenMBVRigidBody(dummy);
#endif


}
//

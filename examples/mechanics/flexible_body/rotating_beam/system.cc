#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/links/joint.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/composite_function.h"

#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"

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
        return 0;
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
  body->addFrame(new NodeFrame("Q",0));
  body->getFrame("K")->enableOpenMBV(0.3);
  body->getFrame("Q")->enableOpenMBV(0.3);

  vector<Vec3> rNod(1);
  rNod[0] = 2.*Kr;

  vector<Mat3xV> TNod(1), PNod(1);
  TNod[0] = T;
  PNod[0] = P;

  body->setNodalRelativePosition(rNod);
  body->setNodalShapeMatrixOfTranslation(TNod);
  body->setNodalShapeMatrixOfRotation(PNod);

  SymMat rrdm(3);
  rrdm(0,0) = J;

  Mat3xV Pdm(2);
  Pdm(0,0) = rho*0.00516935;
  Pdm(1,1) = rho*0.00317895;

  vector<vector<SqrMatV> > PPdm(3,vector<SqrMatV>(3));
  vector<Mat3xV> rPdm(3);
  for(int i=0; i<3; i++) {
    rPdm[i].resize(2);
    for(int j=0; j<3; j++) {
      PPdm[i][j].resize(2);
    }
  }
  PPdm[0][0](0,0) = rho*0.00406;
  PPdm[0][0](1,1) = 0;

  PPdm[1][1](0,0) = 0;
  PPdm[1][1](1,1) = rho*0.00203;

  PPdm[0][1](0,1) = rho*0.00275212;
  PPdm[1][0] = PPdm[0][1].T();
  PPdm[2][1] = PPdm[1][2].T();
  PPdm[2][0] = PPdm[0][2].T();

  rPdm[0](0,0) = rho*0.0668055;
  rPdm[0](1,1) = rho*0.0468815;

  SymMatV Ke(2);
  Ke(0,0) = 2.8442e+06;
  Ke(1,1) = 4249.06;

  body->setMass(m);
  body->setPositionIntegral(m*Kr);
  body->setPositionPositionIntegral(rrdm);
  body->setShapeFunctionIntegral(Pdm);
  body->setShapeFunctionShapeFunctionIntegral(PPdm);
  body->setPositionShapeFunctionIntegral(rPdm);
  body->setStiffnessMatrix(Ke);

  if(stiffening==1) {
    std::vector<SqrMatV> K0om(3);
    for(unsigned int i=0; i<K0om.size(); i++)
      K0om[i].resize(2);
    K0om[2](1,1) = rho*0.00242247;

    body->setGeometricStiffnessMatrixDueToAngularVelocity(K0om);
  }
  else if(stiffening==2) {
    vector<SqrMatV> Kn1(2);
    vector<vector<SqrMatV> > Kn2(2);
    Kn2[0].resize(2);
    Kn2[1].resize(2);

    Kn1[0].resize() = SqrMatV("[186811, 0; 0, 304.802]");
    Kn1[1].resize() = SqrMatV("[0, 98650.5; 304.802, 0]");

    Kn2[0][0].resize() = SqrMatV("[12772.3, 0; 0, 22.1348]");
    Kn2[0][1].resize() = SqrMatV("[0, 2432.46; 2432.46, 0]");
    Kn2[1][0].resize() = SqrMatV("[0, 2432.46; 2432.46, 0]");
    Kn2[1][1].resize() = SqrMatV("[22.1348, 0; 0, 10078.5]");

    body->setNonlinearStiffnessMatrixOfFirstOrder(Kn1);
    body->setNonlinearStiffnessMatrixOfSecondOrder(Kn2);
  }

  body->setRotation(new CompositeFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>("[0;0;1]"), new Angle));

  body->setPlotFeature(generalizedPosition, true);
  body->setPlotFeature(generalizedVelocity, true);
  body->getFrame("K")->setPlotFeature(position, true);
  body->getFrame("K")->setPlotFeature(angle, true);
  body->getFrame("K")->setPlotFeature(velocity, true);
  body->getFrame("K")->setPlotFeature(angularVelocity, true);
  body->getFrame("K")->setPlotFeature(acceleration, true);
  body->getFrame("K")->setPlotFeature(angularAcceleration, true);
}

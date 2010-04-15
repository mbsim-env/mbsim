#include "system.h"
#include "mbsim/constraint.h"
#include "mbsim/special_body.h"
#include "mbsim/environment.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/kinetic_excitation.h"


#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public Function1<fmatvec::Vec, double> {
  public:
    fmatvec::Vec operator()(const double& t, const void * =NULL) { 
      double t0 = 1;
      double t1 = 1.5;
      double M0 = 0*0.2;
      Vec M(1);
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

  double m1 = 0.5;
  double m3 = 0.5;
  double m2 = 0.5;
  double a = 0.075; 
  double b = 0.4; 
  double c = 0.2; 
  double d = 0.4; 
  double r = 0.1; 
  double phi1 = 0.0; 
  double J1 = 1.0/2.0*m1*r*r; 
  double J2 = 1.0/12.0*m2*b*b;
  double J3 = 1.0/12.0*m3*c*c; 

  Vec  Kr(3);   
  SymMat Theta(3);

  Kr(0) = a;
  Kr(2) = 0.02;

  SpecialBody* body1 = new SpecialBody("body1");
  addObject(body1);
  body1->addFrame("Q", Kr,SqrMat(3,EYE));

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));
  body1->setMass(m1);
  Theta(2,2)=J1;
  body1->setInertiaTensor(Theta);
  body1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  body1->setInitialGeneralizedPosition(Vec(1,INIT,phi1)); 

  Kr(0) = d;
  Kr(2) = 0.02;
  addFrame("Q",Kr,SqrMat(3,EYE) );  

  SpecialBody* body2 = new SpecialBody("body2");
  addObject(body2);
  Kr(0) = b/2;
  Kr(2) = 0;
  body2->addFrame("P",-Kr,SqrMat(3,EYE));
  body2->addFrame("Q", Kr,SqrMat(3,EYE));
  body2->getFrame("Q")->enableOpenMBV(0.3);
  body2->setFrameOfReference(body1->getFrame("Q"));
  body2->setFrameForKinematics(body2->getFrame("P"));
  body2->setMass(m2);
  Theta(2,2)=J2;
  body2->setInertiaTensor(Theta);
  body2->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  SpecialBody* body3 = new SpecialBody("body3");
  addObject(body3);
  body3->setFrameOfReference(getFrame("I"));
  body3->setFrameForKinematics(body3->getFrame("C"));
  body3->setMass(m3);
  Theta(2,2)=J3;
  body3->setInertiaTensor(Theta);
  body3->setTranslation(new LinearTranslation(Vec("[1;0;0]")));

  Constraint* constraint = new Constraint1("C",body1,body2,body3,body2->getFrame("Q"),body3->getFrame("C"));
  addObject(constraint);

  KineticExcitation *load = new KineticExcitation("Motor");
  addLink(load);
  load->setMoment("[0;0;1]",new Moment);
  load->connect(body1->getFrame("C"));

#if HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum* dummy = new OpenMBV::Frustum;
  dummy->setBaseRadius(r);
  dummy->setTopRadius(r);
  dummy->setHeight(0.02);
  dummy->setStaticColor(0.1);
  body1->setOpenMBVRigidBody(dummy);
  OpenMBV::Cuboid* dummy1 = new OpenMBV::Cuboid;
  dummy1->setLength(b,0.02,0.02);
  dummy1->setStaticColor(0.5);
  body2->setOpenMBVRigidBody(dummy1);
  OpenMBV::Cuboid* dummy2 = new OpenMBV::Cuboid;
  dummy2->setLength(c,c/2,0.02);
  dummy2->setStaticColor(0.8);
  body3->setOpenMBVRigidBody(dummy2);
#endif


}
//

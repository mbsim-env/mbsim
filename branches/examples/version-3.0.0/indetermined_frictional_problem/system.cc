#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/point.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contact.h"
#include "mbsim/environment.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"


#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/compoundrigidbody.h"
#include "openmbvcppinterface/ivbody.h"
#include "openmbvcppinterface/arrow.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

extern bool rigidContact;

class Friction : public MBSim::Function1<double,double> {
  public:
    //Friction(double mu0_, double mu1_, double mu2_, double k_) : mu0(mu0_), mu1(mu1_), mu2(mu2_), k(k_) {}
    Friction() {}

    virtual ~Friction() {}

    double operator()(const double& gT, const void*) { return 0.1+0.15*exp(-100*gT); }

  protected:
    double mu0, mu1, mu2, k;
};


class Force : public Function1<fmatvec::Vec, double> {
  double F0, t0, t1, t2, t3;
  public:
    Force(double F0_, double t0_, double t1_, double t2_, double t3_) : F0(F0_), t0(t0_), t1(t1_), t2(t2_), t3(t3_)  {}
    fmatvec::Vec operator()(const double& t, const void * =NULL) {
      Vec F(1);
      if(t<=t0)
	F(0) = 0;
      else if(t<=t1)
	F(0) = (t-t0)/(t1-t0)*F0;
      else if(t<=t2)
	F(0) = F0;
      else if(t<=t3)
	F(0) = F0 - (t-t2)/(t3-t2)*F0;
      else
	F(0) = 0;
      return F;
    };
};

class Schwerpunkt : public LinkMechanics {
  private:
    vector<RigidBody*> body;
    vector<double> mass;
    Frame *frame;
  public:
    Schwerpunkt(const std::string &name) : LinkMechanics(name) {
      frame = new Frame("S");
      connect(frame);
    }
    void setBodies(vector<RigidBody*> body_) { body = body_; frame->setParent(body[0]); } 
    void setMasses(vector<double> mass_) { mass = mass_; } 
    virtual void updateh(double t) {
      double m = 0;
      for(unsigned int i=0; i<mass.size(); i++) {
	m += mass[i];
      }
      Vec tmp(3);
      for(unsigned int i=0; i<body.size(); i++) {
	tmp += mass[i]*body[i]->getFrame("C")->getPosition();
      }
      frame->setPosition(tmp/m);
      WF[0](1) = -m*9.81;
    }
    virtual void updateg(double) {}
    virtual void updategd(double) {}
    //virtual void init(InitStage stage);
    bool isActive() const { return true; }
    bool gActiveChanged() { return false; }
    //virtual void plot(double t, double dt = 1);
    void setOpenMBVForceArrow(OpenMBV::Arrow *arrow) {
      std::vector<bool> which; which.resize(1, true);
      LinkMechanics::setOpenMBVForceArrow(arrow, which);
    }
};

System::System(const string &projectName) : MySolver(projectName) {
  // Gravitation
  bool twoContacts = true;
  bool Coulomb = false;
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  // Parameters
  double b2 = 2./2;
  double h2 = 1./2;
  double m2 = 1;
  SymMat Theta2(3);
  Theta2(2,2) = 1./12.*m2*(b2*b2+h2*h2);
  double gam, e, F0, x0;

  double delta = 1;
  double sl = 0.1;

  F0 = 3.51;
  F0 = 3.55;

  double t0 = 1;
  double t1 = 3;
  double t2 = 4;
  double t3 = 5;
  Vec r(3);

 
  RigidBody* box2 = new RigidBody("Box2");
  addObject(box2);

  r(0) = -b2/2;
  r(1) = h2/2;
  box2->addFrame("K2",r,SqrMat(3,EYE));

  box2->setFrameOfReference(getFrame("I"));
  box2->setFrameForKinematics(box2->getFrame("C"));
  box2->setMass(m2);
  box2->setInertiaTensor(Theta2);
  box2->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));
  if(twoContacts)
    box2->setRotation(new RotationAboutZAxis);

  Line* line = new Line("Boden");
  addContour(line,Vec(3),BasicRotAKIz(-M_PI/2));

  line = new Line("Box2O");
  r(0) = 0;
  r(1) = h2/2.;
  box2->addContour(line,r,BasicRotAKIz(-M_PI/2));

  Point* point = new Point("UL");
  if(twoContacts) {
  r(0) = -b2/2;
  r(1) = -h2/2;
  }
  else {
  r(0) = 0.01;
  r(1) = -h2/2;
  }
  box2->addContour(point,r,SqrMat(3,EYE));
  point = new Point("UR");
  r(0) = b2/2;
  r(1) = -h2/2;
  box2->addContour(point,r,SqrMat(3,EYE));


  if(twoContacts) {
  Vec q2(3);
  q2(1) = h2/2;
  box2->setInitialGeneralizedPosition(q2);
  }
  else {
  Vec q2(2);
  q2(1) = h2/2;
  box2->setInitialGeneralizedPosition(q2);
  }

  Contact *rc = new Contact("Contact2UL");
  rc->connect(getContour("Boden"), box2->getContour("UL"));
  addLink(rc);
  if(rigidContact) {
    rc->setContactForceLaw(new UnilateralConstraint);
    rc->setContactImpactLaw(new UnilateralNewtonImpact);
    rc->setFrictionForceLaw(new PlanarStribeckFriction(new Friction));
    rc->setFrictionImpactLaw(new PlanarStribeckImpact(new Friction));
    if(Coulomb) {
    rc->setFrictionForceLaw(new PlanarCoulombFriction(0.25));
    rc->setFrictionImpactLaw(new PlanarCoulombImpact(0.25));
    }
  } 
  else {
    rc->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e4,1e4)));
    rc->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedStribeckFriction(new Friction,0.001)));
  }
  //rc->enableOpenMBVContactPoints(1);
  OpenMBV::Arrow *arrow = new OpenMBV::Arrow;
   arrow->setScaleLength(sl);
   arrow->setDiameter(0.1);
   arrow->setHeadDiameter(0.2);
   arrow->setHeadLength(0.2);
  rc->setOpenMBVNormalForceArrow(arrow);
  arrow = new OpenMBV::Arrow;
   arrow->setScaleLength(sl);
   arrow->setDiameter(0.1);
   arrow->setHeadDiameter(0.2);
   arrow->setHeadLength(0.2);
  rc->setOpenMBVFrictionArrow(arrow);

  if(twoContacts) {
  rc = new Contact("Contact2UR");
  rc->connect(getContour("Boden"), box2->getContour("UR"));
  addLink(rc);
  if(rigidContact) {
    rc->setContactForceLaw(new UnilateralConstraint);
    rc->setContactImpactLaw(new UnilateralNewtonImpact);
    rc->setFrictionForceLaw(new PlanarStribeckFriction(new Friction));
    rc->setFrictionImpactLaw(new PlanarStribeckImpact(new Friction));
    if(Coulomb) {
    rc->setFrictionForceLaw(new PlanarCoulombFriction(0.25));
    rc->setFrictionImpactLaw(new PlanarCoulombImpact(0.25));
    }
  } 
  else {
    rc->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e4,1e4)));
    rc->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedStribeckFriction(new Friction,0.001)));
  }
  //rc->enableOpenMBVContactPoints(1);
  arrow = new OpenMBV::Arrow;
   arrow->setScaleLength(sl);
   arrow->setDiameter(0.1);
   arrow->setHeadDiameter(0.2);
   arrow->setHeadLength(0.2);
  rc->setOpenMBVNormalForceArrow(arrow);
  arrow = new OpenMBV::Arrow;
   arrow->setScaleLength(sl);
   arrow->setDiameter(0.1);
   arrow->setHeadDiameter(0.2);
   arrow->setHeadLength(0.2);
  rc->setOpenMBVFrictionArrow(arrow);
  }


#ifdef HAVE_OPENMBVCPPINTERFACE

  OpenMBV::Cuboid* obj;

  obj = new OpenMBV::Cuboid;
  obj->setLength(b2,h2,h2); 
  box2->setOpenMBVRigidBody(obj);

  //  r.init(0);
  //  r(1) = b;
  //  r(0) = -a;
  //  hebel->addFrame("Q",r,SqrMat(3,EYE),hebel->getFrame("K"));
   KineticExcitation* ke = new KineticExcitation("Kraft");
   addLink(ke);
   ke->connect(box2->getFrame("K2"));
   ke->setForce("[0.866;-0.5;0]", new Force(F0,t0,t1,t2,t3));
   arrow = new OpenMBV::Arrow;
   arrow->setScaleLength(sl);
   arrow->setDiameter(0.1);
   arrow->setHeadDiameter(0.2);
   arrow->setHeadLength(0.2);
  ke->setOpenMBVForceArrow(arrow);
  //
  //  obj = new OpenMBV::IvBody;
  //  obj->setIvFileName("wrl/Hebel.wrl");
  //  hebel->setOpenMBVFrameOfReference(hebel->getFrame("K"));
  //  hebel->setOpenMBVRigidBody(obj);
  //  obj->setInitialTranslation(0,0.02,0);
  
  Schwerpunkt *sp = new Schwerpunkt("Schwerpunkt");
  addLink(sp);
  vector<RigidBody*> body;
  vector<double> mass;
  body.push_back(box2);
  mass.push_back(m2);
  sp->setBodies(body);
  sp->setMasses(mass);
  arrow = new OpenMBV::Arrow;
  arrow->setScaleLength(sl);
  arrow->setDiameter(0.1);
  arrow->setHeadDiameter(0.2);
  arrow->setHeadLength(0.2);
  sp->setOpenMBVForceArrow(arrow);

#endif
}


#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contact.h"
#include "mbsimControl/actuator.h"
#include "mbsim/utils/function_library.h"
#include "mbsim/environment.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsimControl/function_sensor.h"
#include "mbsim/joint.h" 
#include "mbsim/constraint.h" 
#include "mbsim/spring_damper.h"
#include "mbsimPowertrain/differential_gear.h"
#include "mbsimPowertrain/cardan_shaft.h"
#include "mbsim/kinetic_excitation.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/ivbody.h"
#endif

extern bool rigidContacts;
extern bool rigidJoints;
extern bool useDAE;
extern double ue;

using namespace MBSim;
using namespace MBSimPowertrain;
using namespace MBSimControl;
using namespace fmatvec;

double mK = 1600;
double mR = 10;
double g0 = 9.81;
double mu = 1.07;

class Fahrwiderstand : public Function1<fmatvec::Vec, double> {
  private:
    RigidBody *body;
  public:
    Fahrwiderstand(RigidBody *body_) : body(body_) {}

    fmatvec::Vec operator()(const double& tVal, const void * =NULL) {
      fmatvec::Vec F(1);
      double v = abs(body->getu()(0));
      double vkmh = v*3.6;
      if(vkmh<0.1)
	return F;

      double rho = 1.2;
      double A = 2.3;
      double cx = 0.3;

      double F_Luft = 0.5 * rho * A * cx * v*v;
      double cr = 0.9/100;
      double F_Rad = (mK+4*mR) * g0 * cr;
      F(0) = F_Luft + F_Rad;
      return F;
    };
};

class Moment : public Function1<fmatvec::Vec, double> {
    RigidBody *shaft,*fzg;
    double P, M_max, Om_eck, Om_max, t0;
    double i, M_An, M_Ab, Om_Ab, Om_An, v_Fzg;
    bool isAntrieb;
  public:
    Moment(RigidBody *shaft_, RigidBody *fzg_, bool isAntrieb_) : shaft(shaft_), fzg(fzg_), isAntrieb(isAntrieb_) {
      P = 60e3;
      t0 = 2;
      Om_max = 11400./30*M_PI/ue;
      Om_eck = 3440./30*M_PI/ue;
      M_max = P/Om_eck;
    }

    fmatvec::Vec operator()(const double& tVal, const void * =NULL) {
      fmatvec::Vec M(1);
      if(tVal<t0)
	return M;

      Om_Ab = abs(shaft->getuRel()(0));
      v_Fzg = fzg->getu()(0)*3.6; // km/h

      i = v_Fzg < 100 ? 12 : 5;
      i = 1;

      Om_An = Om_Ab*i;

      if(Om_An<Om_eck)
	M_An = M_max;
      else if(Om_An<Om_max)
	M_An = P/Om_An;
      else
	M_An = 0;

      M_Ab= M_An*i;

      M(0) = isAntrieb ? M_An : M_Ab;
      return M;
    } 
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-g0;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double w = 3;
  double b = 2;
  double d = 1.5;
  double h = 0.5;
  double e = w-d;
  double r = 0.35;
  double bR = 0.2;
  double cs = 2e4;
  double ds = 1e3;
  double l0 = 0.55;

  double cc = 1e6;
  double dc = 1e3;


  SymMat ThetaK(3);
  SymMat ThetaR(3);
  ThetaK(0,0) = 100;
  ThetaK(1,1) = 100;
  ThetaK(2,2) = 100;
  ThetaR(2,2) = 1.6;
  ThetaR(1,1) = 1.6;
  ThetaR(0,0) = 1.6;

  SqrMat ASP(3,EYE);

  RigidBody *karosserie = new RigidBody("Karosserie");
  addObject(karosserie);

  karosserie->setMass(mK);
  karosserie->setInertiaTensor(ThetaK);

  Vec SrSP(3);

  karosserie->setTranslation(new LinearTranslation(SqrMat(3,EYE)));
  karosserie->setRotation(new CardanAngles);
  karosserie->setFrameOfReference(getFrame("I"));
  karosserie->setFrameForKinematics(karosserie->getFrame("C"));
  karosserie->setPlotFeature(stateDerivative,enabled);

  SrSP(0) = d;
  SrSP(1) = -h+r;
  SrSP(2) = -b/2+bR/2;
  karosserie->addFrame("VL",SrSP,ASP);
  SrSP(2) = b/2-bR/2;
  karosserie->addFrame("VR",SrSP,ASP);

  SrSP(1) = -h+r+0.3;
  SrSP(2) = -b/2+bR/2;
  karosserie->addFrame("FVL",SrSP,ASP);
  SrSP(2) = b/2-bR/2;
  karosserie->addFrame("FVR",SrSP,ASP);

  SrSP(0) = -e;
  SrSP(1) = -h+r;
  SrSP(2) = -b/2+bR/2;
  karosserie->addFrame("HL",SrSP,ASP);
  SrSP(2) = +b/2-bR/2;
  karosserie->addFrame("HR",SrSP,ASP);

  SrSP(1) = -h+r+0.3;
  SrSP(2) = -b/2+bR/2;
  karosserie->addFrame("FHL",SrSP,ASP);
  SrSP(2) = +b/2-bR/2;
  karosserie->addFrame("FHR",SrSP,ASP);
#ifdef HAVE_OPENMBVCPPINTERFACE
  karosserie->getFrame("FHL")->enableOpenMBV(0.3);
#endif

  RigidBody *vl = new RigidBody("VorderradLinks");
  addObject(vl);

  vl->setMass(mR);
  vl->setInertiaTensor(ThetaR);

  vl->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  vl->setTranslation(new LinearTranslation(Vec("[0;1;0]")));
  vl->setFrameOfReference(karosserie->getFrame("VL"));
  vl->setFrameForKinematics(vl->getFrame("C"));

  SpringDamper *sd = new SpringDamper("FederVL");
  addLink(sd);
  sd->setForceFunction(new LinearSpringDamperForce(cs,ds,l0));
  sd->connect(karosserie->getFrame("FVL"),vl->getFrame("C"));


  RigidBody *vr = new RigidBody("VorderradRechts");
  addObject(vr);

  vr->setMass(mR);
  vr->setInertiaTensor(ThetaR);

  vr->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  vr->setTranslation(new LinearTranslation(Vec("[0;1;0]")));
  vr->setFrameOfReference(karosserie->getFrame("VR"));
  vr->setFrameForKinematics(vr->getFrame("C"));

  sd = new SpringDamper("FederVR");
  addLink(sd);
  sd->setForceFunction(new LinearSpringDamperForce(cs,ds,l0));
  sd->connect(karosserie->getFrame("FVR"),vr->getFrame("C"));

  RigidBody *hl = new RigidBody("HinterradLinks");
  addObject(hl);

  hl->setMass(mR);
  hl->setInertiaTensor(ThetaR);

  hl->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  hl->setTranslation(new LinearTranslation(Vec("[0;1;0]")));
  hl->setFrameOfReference(karosserie->getFrame("HL"));
  hl->setFrameForKinematics(hl->getFrame("C"));

  sd = new SpringDamper("FederHL");
  addLink(sd);
  sd->setForceFunction(new LinearSpringDamperForce(cs,ds,l0));
  sd->connect(karosserie->getFrame("FHL"),hl->getFrame("C"));

  RigidBody *hr = new RigidBody("HinterradRechts");
  addObject(hr);

  hr->setMass(mR);
  hr->setInertiaTensor(ThetaR);

  hr->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  hr->setTranslation(new LinearTranslation(Vec("[0;1;0]")));

  hr->setFrameOfReference(karosserie->getFrame("HR"));
  hr->setFrameForKinematics(hr->getFrame("C"));

  sd = new SpringDamper("FederHR");
  addLink(sd);
  sd->setForceFunction(new LinearSpringDamperForce(cs,ds,l0));
  sd->connect(karosserie->getFrame("FHR"),hr->getFrame("C"));

  SrSP.init(0);
  CircleSolid *circle = new CircleSolid("Reifen");
  circle->setRadius(r);
  vl->addContour(circle,SrSP,ASP);

  circle = new CircleSolid("Reifen");
  circle->setRadius(r);
  hl->addContour(circle,SrSP,ASP);

  circle = new CircleSolid("Reifen");
  circle->setRadius(r);
  vr->addContour(circle,SrSP,ASP);

  circle = new CircleSolid("Reifen");
  circle->setRadius(r);
  hr->addContour(circle,SrSP,ASP);

  Plane *plane = new Plane("Ebene");
  double phi = M_PI*0.5; 
  SqrMat A(3);
  A(0,0) = cos(phi);
  A(0,1) = -sin(phi);
  A(1,1) = cos(phi);
  A(1,0) = sin(phi);
  A(2,2) = 1;
  addContour(plane,SrSP,A);

  Contact *c = new Contact("KontaktVorneLinks"); 
  if(rigidContacts) {
    c->setContactForceLaw(new UnilateralConstraint);
    c->setContactImpactLaw(new UnilateralNewtonImpact);
    c->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    c->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  } else {
    c->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cc,dc)));
    c->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  c->connect(getContour("Ebene"),vl->getContour("Reifen"));
  addLink(c);

  c = new Contact("KontaktHintenLinks"); 
  if(rigidContacts) {
    c->setContactForceLaw(new UnilateralConstraint);
    c->setContactImpactLaw(new UnilateralNewtonImpact);
    c->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    c->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  } else {
    c->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cc,dc)));
    c->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  c->connect(getContour("Ebene"),hl->getContour("Reifen"));
  addLink(c);

  c = new Contact("KontaktVorneRechts"); 
  if(rigidContacts) {
    c->setContactForceLaw(new UnilateralConstraint);
    c->setContactImpactLaw(new UnilateralNewtonImpact);
    c->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    c->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  } else {
    c->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cc,dc)));
    c->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  c->connect(getContour("Ebene"),vr->getContour("Reifen"));
  addLink(c);

  c = new Contact("KontaktHintenRechts"); 
  if(rigidContacts) {
    c->setContactForceLaw(new UnilateralConstraint);
    c->setContactImpactLaw(new UnilateralNewtonImpact);
    c->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    c->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  } else {
    c->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(cc,dc)));
    c->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
  }
  c->connect(getContour("Ebene"),hr->getContour("Reifen"));
  addLink(c);

  Vec q0(6);
  q0(1)= h+0.00;
  karosserie->setInitialGeneralizedPosition(q0);
  Vec u0(6);
  u0(0)= 0;
  karosserie->setInitialGeneralizedVelocity(u0);

  Actuator *actuator;

  FunctionSensor *sensor;

  DifferentialGear::Data data;
  data.lengthLeftOutputShaft = 0.1; 
  data.lengthRightOutputShaft = 0.1; 

  DifferentialGear* differentialGear = new DifferentialGear("DifferentialGear",data);
  addGroup(differentialGear);
 
  Vec rSD(3);
  rSD(0) = -w/2;
  rSD(1) = -h+r;
  karosserie->addFrame("D",rSD,BasicRotAKIy(0));
  static_cast<RigidBody*>(differentialGear->getObject("Housing"))->setFrameOfReference(karosserie->getFrame("D"));

  CardanShaft* psR = new CardanShaft("CardanShaftRechts");
  addGroup(psR);
  psR->getInputShaft()->setFrameOfReference(differentialGear->getRightOutputShaft()->getFrame("Q"));


  CardanShaft* psL = new CardanShaft("CardanShaftLinks");
  addGroup(psL);
  psL->getInputShaft()->setFrameOfReference(differentialGear->getLeftOutputShaft()->getFrame("Q"));

  rSD.init(0);
  rSD(2) = -bR/2;
  hr->addFrame("K",rSD,BasicRotAKIz(M_PI/2));
#ifdef HAVE_OPENMBVCPPINTERFACE
  hr->getFrame("K")->enableOpenMBV(0.3);
#endif

  rSD.init(0);
  rSD(2) = bR/2;
  hl->addFrame("K",rSD,BasicRotAKIy(-M_PI)*BasicRotAKIz(M_PI/2));
#ifdef HAVE_OPENMBVCPPINTERFACE
  hl->getFrame("K")->enableOpenMBV(0.3);
#endif

  // Decide, wheter to use DAE- or minimal-form
  if(useDAE) {
    // Use DAE-form

    Joint *joint2 = new Joint("AnbindungRadRechts");
    addLink(joint2);
    joint2->setForceDirection(Mat("[1,0,0; 0,1,0; 0,0,1]"));
    joint2->setMomentDirection(Mat("[1,0,0; 0,1,0; 0,0,1]"));
    joint2->connect(hr->getFrame("K"),psR->getOutputShaft()->getFrame("Q"));

    if(rigidJoints) {
      joint2->setForceLaw(new BilateralConstraint);
      joint2->setImpactForceLaw(new BilateralImpact);
      joint2->setMomentLaw(new BilateralConstraint);
      joint2->setImpactMomentLaw(new BilateralImpact);
    } 
    else {
      joint2->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1e6,1000)));
      joint2->setMomentLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1e5,100)));
    }

    joint2 = new Joint("AnbindungRadLinks");
    addLink(joint2);
    joint2->setForceDirection(Mat("[1,0,0; 0,1,0; 0,0,1]"));
    joint2->setMomentDirection(Mat("[1,0,0; 0,1,0; 0,0,1]"));
    joint2->connect(hl->getFrame("K"),psL->getOutputShaft()->getFrame("Q"));

    if(rigidJoints) {
      joint2->setForceLaw(new BilateralConstraint);
      joint2->setImpactForceLaw(new BilateralImpact);
      joint2->setMomentLaw(new BilateralConstraint);
      joint2->setImpactMomentLaw(new BilateralImpact);
    } 
    else {
      joint2->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1e6,1000)));
      joint2->setMomentLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1e5,100)));
    }

  } 
  else {
    // Use minimal-form
    {
      vector<RigidBody*> bd1; 
      vector<RigidBody*> bd2; 
      bd2.push_back(differentialGear->getRightOutputShaft());
      bd2.push_back(psR->getInputShaft());
      bd2.push_back(psR->getIntermediateShaft());
      bd2.push_back(psR->getOutputShaft());
      JointConstraint* jointconstraint = new JointConstraint("CR");
      jointconstraint->setDependentBodiesSecondSide(bd2);
      jointconstraint->setIndependentBody(hr);
      jointconstraint->connect(hr->getFrame("K"),psR->getOutputShaft()->getFrame("Q"));
      addObject(jointconstraint);
      jointconstraint->setForceDirection(Mat(3,3,EYE));
      jointconstraint->setMomentDirection(Mat(3,3,EYE));
    }

    {
      vector<RigidBody*> bd1; 
      vector<RigidBody*> bd2; 
      bd2.push_back(differentialGear->getLeftOutputShaft());
      bd2.push_back(psL->getInputShaft());
      bd2.push_back(psL->getIntermediateShaft());
      bd2.push_back(psL->getOutputShaft());
      JointConstraint* jointconstraint = new JointConstraint("CL");
      jointconstraint->setDependentBodiesSecondSide(bd2);
      jointconstraint->setIndependentBody(hl);
      jointconstraint->connect(hl->getFrame("K"),psL->getOutputShaft()->getFrame("Q"));
      addObject(jointconstraint);
      jointconstraint->setForceDirection(Mat(3,3,EYE));
      jointconstraint->setMomentDirection(Mat(3,3,EYE));
    }
  }

  RigidBody* shaft1 = new RigidBody("DriveShaft");
  addObject(shaft1);

  rSD(1) = -h+r;

  karosserie->addFrame("W",rSD,BasicRotAKIy(M_PI/2));
  shaft1->setFrameOfReference(karosserie->getFrame("W"));
  shaft1->setFrameForKinematics(shaft1->getFrame("C"));

  double r1 = 0.05;
  double l = r1*55;
  double m = 785*l*pow(r1,2)*M_PI;
  shaft1->setMass(m);
  SymMat I(3);
  I(0,0) = m*pow(l,2)/12.0;
  I(1,1) = m*pow(l,2)/12.0;
  I(2,2) = m*pow(l,2)/2.0;
  shaft1->setInertiaTensor(I);
  shaft1->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  rSD.init(0);
  rSD(2) = l/2;
  shaft1->addFrame("Q",rSD,SqrMat(3,EYE));
#ifdef HAVE_OPENMBVCPPINTERFACE
  shaft1->getFrame("Q")->enableOpenMBV(0.3);
#endif

  GearConstraint *constraint = new GearConstraint("C0",shaft1);
  addObject(constraint);
  constraint->addDependency(static_cast<RigidBody*>(differentialGear->getObject("InputShaft")),-differentialGear->getRadiusInputShaft()/r1);

  //actuator = new Actuator("Fahrwiderstand");
  //addLink(load);
  //load->setForceDirection("[-1;0;0]");
  //load->setUserFunction(new Fahrwiderstand(karosserie));
  //load->connect(karosserie->getFrame("C"));
  //
  sensor = new FunctionSensor("MomentSensor");
  addLink(sensor);

  sensor->setFunction(new Moment(shaft1,karosserie,true));
  actuator = new Actuator("EMotor");
  addLink(actuator);
  actuator->setSignal(sensor);
  actuator->setMomentDirection("[1;0;0]");
  actuator->connect(karosserie->getFrame("C"),shaft1->getFrame("C"));

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::IvBody *obj=new OpenMBV::IvBody;
  obj->setInitialRotation(-M_PI/2,0,0);
  obj->setIvFileName("wrl/car.wrl");
  karosserie->setOpenMBVRigidBody(obj);
  obj->setScaleFactor(1);

  OpenMBV::Frustum* cylinder = new OpenMBV::Frustum;
  cylinder->setBaseRadius(r);
  cylinder->setTopRadius(r);
  cylinder->setHeight(bR);
  cylinder->setStaticColor(0.9);
  vl->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,0,bR/2);
  cylinder -> setInitialRotation(0,0,0);

  cylinder = new OpenMBV::Frustum;
  cylinder->setBaseRadius(r);
  cylinder->setTopRadius(r);
  cylinder->setHeight(bR);
  cylinder->setStaticColor(0.9);
  vr->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,0,bR/2);
  cylinder -> setInitialRotation(0,0,0);

  cylinder = new OpenMBV::Frustum;
  cylinder->setBaseRadius(r);
  cylinder->setTopRadius(r);
  cylinder->setHeight(bR);
  cylinder->setStaticColor(0.9);
  hl->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,0,bR/2);
  cylinder -> setInitialRotation(0,0,0);

  cylinder = new OpenMBV::Frustum;
  cylinder->setBaseRadius(r);
  cylinder->setTopRadius(r);
  cylinder->setHeight(bR);
  cylinder->setStaticColor(0.9);
  hr->setOpenMBVRigidBody(cylinder);
  cylinder -> setInitialTranslation(0,0,bR/2);
  cylinder -> setInitialRotation(0,0,0);

  cylinder=new OpenMBV::Frustum;
  cylinder->setTopRadius(r1);
  cylinder->setBaseRadius(r1);
  cylinder->setHeight(l);
  cylinder->setStaticColor(0.1);
  shaft1->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);
#endif

}


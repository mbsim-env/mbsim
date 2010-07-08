#include "woodpecker.h"

#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contour.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/flexible_band.h"
#include "mbsim/contact_kinematics/point_flexibleband.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/rotational_spring_damper.h"

#include <iostream>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/ivbody.h"
#endif
using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimFlexibleBody;

Woodpecker::Woodpecker(const string &projectName) : DynamicSystemSolver(projectName) {

  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // Bewegungsrichtung der Koerper --------------
  Mat JT(3,2);
  JT(0,0) = 1;
  JT(1,1) = 1;
  Vec JR(3);
  JR(2) = 1;

  // Stange -------------------------------------
  int Elements = 5;
  double     L = 1.0;
  double     r = 10.0e-3;
  double spiel = 0.7e-3;
  double     R =  r + spiel/2.;
  double hoehe = R/2.;

  FlexibleBody1s21RCM *balken = new FlexibleBody1s21RCM("Balken",true);
  balken->setFrameOfReference(this->getFrame("I"));

  balken->setNumberElements(Elements); 
  balken->setLength(L);
  balken->setEModul(7.5e09);

  balken->setCrossSectionalArea(r*r*M_PI);
  balken->setMomentInertia(r*r*r*r/12.);
  balken->setLehrDamping(0.10);
  balken->setDensity(2.3e3);

  balken->initRelaxed(90./180.*M_PI);
  cout << "Balken.q0 = " << trans( balken->getq0() ) << endl;

  addObject(balken);

  // inertiale Einspannung -----------------------------
  ContourPointData cpdata;
  cpdata.getLagrangeParameterPosition() = Vec(1,INIT,0.);
  cpdata.getContourParameterType() = CONTINUUM;
  balken->addFrame("RJ",cpdata);
  Joint *joint = new Joint("Clamping");
  joint->connect(this->getFrame("I"),balken->getFrame("RJ")); 
  joint->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->setImpactForceLaw(new BilateralImpact);
  joint->setMomentDirection("[0; 0; 1]");
  joint->setMomentLaw(new BilateralConstraint);
  joint->setImpactMomentLaw(new BilateralImpact);
  this->addLink(joint);

  Vec nodes(Elements+1);
  for(int i=0;i<=Elements;i++) nodes(i) = i*L/Elements;
  FlexibleBand *top = new FlexibleBand("Top");
  top->setNodes(nodes);
  top->setWidth(r);
  top->setCn(Vec("[-1.;0.]"));
  top->setAlphaStart(0.);
  top->setAlphaEnd(L);  
  top->setNormalDistance(r);
  balken->addContour(top);
  FlexibleBand *bot = new FlexibleBand("Bot");
  bot->setNodes(nodes);
  bot->setWidth(r);
  bot->setCn(Vec("[1.;0.]"));
  bot->setAlphaStart(0.);
  bot->setAlphaEnd(L);  
  bot->setNormalDistance(r);
  balken->addContour(bot);

  SymMat Theta(3,INIT,0.0);
  Vec WrOS(3,INIT,0.0);
  // Muffe -------------------------------------
  double FDPunkt = 27.e-3;
  double yMuffe0 =  0.925;
  double  mMuffe = 10.e-3;
  double  JMuffe =  5.e-6;
  // 

  RigidBody *muffe = new RigidBody("Muffe");
  muffe->setMass(mMuffe);
  Theta(2,2) = JMuffe;
  muffe->setInertiaTensor(Theta);
  this->addObject(muffe);

  WrOS(1) = yMuffe0;
  this->addFrame("B",WrOS,SqrMat(3,EYE),this->getFrame("I"));
  muffe->setFrameOfReference(this->getFrame("B"));
  muffe->setFrameForKinematics(muffe->getFrame("C"));
  muffe->setTranslation(new LinearTranslation(JT));
  muffe->setRotation(new RotationAboutFixedAxis(JR));
//  muffe->setRotation(new RotationAboutZAxis());

  muffe->setInitialGeneralizedPosition(spiel*Vec("[0.4;0.0;0.0]"));
  muffe->setInitialGeneralizedVelocity(      Vec("[0.0;0.0;0.0]"));

  UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
  UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
  double mu = 0.2;
  SpatialCoulombFriction *coulFriction = new SpatialCoulombFriction(mu);
  SpatialCoulombImpact   *coulImptact  = new SpatialCoulombImpact(mu);

  for(int i=0;i<4;i++) {

    UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
    UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
    SpatialCoulombFriction *coulFriction = new SpatialCoulombFriction(mu);
    SpatialCoulombImpact   *coulImptact  = new SpatialCoulombImpact(mu);

    stringstream name;
    name << "PM" << i;
    Point* pMuffe = new Point(name.str());
    name.clear();
    name << "KontaktMB" << i; 
    Contact *contact = new Contact(name.str());
    Vec KrSPMuffe(3);

    switch(i){
      case 0: contact->connect(pMuffe,balken->getContour("Top"));
              KrSPMuffe(0) =  R;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 1: contact->connect(pMuffe,balken->getContour("Top"));
              KrSPMuffe(0) =  R;   KrSPMuffe(1) = -hoehe/2.;
              break;
      case 2: contact->connect(pMuffe,balken->getContour("Bot"));
              KrSPMuffe(0) = -R;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 3: contact->connect(pMuffe,balken->getContour("Bot"));
              KrSPMuffe(0) = -R;   KrSPMuffe(1) = -hoehe/2.;
              break;
    }
    muffe->addContour(pMuffe,KrSPMuffe,SqrMat(3,EYE));
    contact->setContactForceLaw  (cntForceLaw);
    contact->setContactImpactLaw (impForceLaw );
    contact->setFrictionForceLaw (coulFriction);
    contact->setFrictionImpactLaw(coulImptact );
    addLink(contact);
  }

  // Drehpunkt der Feder
  Vec KrSPFederDrehpunkt(3);
  KrSPFederDrehpunkt(0) = FDPunkt;
  muffe->addFrame("Drehpunkt",KrSPFederDrehpunkt,SqrMat(3,EYE));

  // Specht --------------------------------------
  double  mSpecht = 100.e-3;
  double  JSpecht =  0.4e-3;

  RigidBody *specht = new RigidBody("Specht");
  specht->setFrameOfReference(muffe->getFrame("Drehpunkt"));
  specht->setRotation(new RotationAboutFixedAxis(JR));
//  specht->setRotation(new RotationAboutZAxis());
  this->addObject(specht);

  Vec MrDrehpunkt(3); /* wird von Schwerpunkt aus bemast */
  MrDrehpunkt(0) = -2.0 * FDPunkt;
  specht->addFrame("D",MrDrehpunkt,SqrMat(3,EYE),specht->getFrame("C"));
  specht->setFrameForKinematics(specht->getFrame("D"));

  Vec SrSchabelspitze(3);
  SrSchabelspitze(1) =  50.e-3;
  SrSchabelspitze(0) = 1.0 * ( - 3*FDPunkt  + r + 1.0e-3) ;

  specht->setMass(mSpecht);
  Theta(2,2) = JSpecht;
  specht->setInertiaTensor(Theta);

  Point* schnabel = new Point("Schabel");
  specht->addContour(schnabel,SrSchabelspitze,SqrMat(3,EYE));

  RelativeRotationalSpringDamper *feder = new RelativeRotationalSpringDamper("Drehfeder");
  feder->connect(muffe->getFrame("Drehpunkt"),specht->getFrame("D"));
  feder->setRelativeBody(specht);
  double cDF = 0.5;
  feder->setForceFunction(new LinearSpringDamperForce(cDF,0.0,0.0));
  addLink(feder);

  Contact *contact = new Contact("SchnabelKontakt");
  contact->connect(schnabel,balken->getContour("Top"));
  contact->setContactForceLaw  (cntForceLaw );
  contact->setContactImpactLaw (impForceLaw );
  contact->setFrictionForceLaw (coulFriction);
  contact->setFrictionImpactLaw(coulImptact );
  addLink(contact);

  specht->setInitialGeneralizedPosition(Vec(1,INIT, 0.0));
  specht->setInitialGeneralizedVelocity(Vec(1,INIT,-5.0));

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid=new OpenMBV::SpineExtrusion;
  int spineDiscretisation = 4;
  cuboid->setNumberOfSpinePoints(Elements*spineDiscretisation+1); // resolution of visualisation
  cuboid->setStaticColor(0.6); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>; // clockwise ordering, no doubling for closure
  int circDiscretisation = 36;
  for(int i=0;i<=circDiscretisation;i++) {
    double phi = 2*M_PI/circDiscretisation*i;
    OpenMBV::PolygonPoint *corner = new OpenMBV::PolygonPoint(r*cos(phi),r*sin(phi),1);
    rectangle->push_back(corner);
  }

  cuboid->setContour(rectangle);
  balken->setOpenMBVSpineExtrusion(cuboid);

  OpenMBV::IvBody* muffeMBV = new OpenMBV::IvBody;
  muffeMBV->setIvFileName("../objects/muffe.wrl");
  muffeMBV->setInitialRotation( 0, 0, M_PI/2. );
  muffeMBV->setScaleFactor(R);
  muffe->setOpenMBVRigidBody(muffeMBV); 

  OpenMBV::IvBody* spechtMBV = new OpenMBV::IvBody;
  spechtMBV->setIvFileName("../objects/specht.wrl");
  spechtMBV->setInitialTranslation( SrSchabelspitze(0), SrSchabelspitze(1), SrSchabelspitze(2));
  spechtMBV->setInitialRotation(M_PI/2., 0, 0);
  spechtMBV->setScaleFactor(0.05);
  specht->setOpenMBVRigidBody(spechtMBV); 
#endif
}


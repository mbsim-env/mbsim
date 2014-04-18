#include "woodpecker.h"

#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contour.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contact_kinematics/point_line.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/ivbody.h"
#endif

#include <iostream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

Woodpecker::Woodpecker(const string &projectName) : DynamicSystemSolver(projectName) {

  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // Bewegungsrichtung der Koerper --------------
  Mat JT(3,2);
  JT(0,0) = 1;
  JT(1,1) = 1;
  Vec3 JR;
  JR(2) = 1;

  // Stange -------------------------------------
  double     r = 10.0e-3;
  double spiel = 0.7e-3;
  double     R =  r + spiel/2.;
  double hoehe = R/2.;

  Line *lL = new Line("LineL");
  Line *lR = new Line("LineR");

  this->addFrame(new FixedRelativeFrame("LineL", r/2.*Vec("[1.0;0.0;0.0]"), BasicRotAKIz(0)));
  this->addFrame(new FixedRelativeFrame("LineR",-r/2.*Vec("[1.0;0.0;0.0]"), BasicRotAKIz(M_PI)));
  lL->setFrameOfReference(this->getFrame("LineL"));
  lR->setFrameOfReference(this->getFrame("LineR"));
  this->addContour(lL);
  this->addContour(lR);

  SymMat Theta(3);
  Vec WrOS(3);
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
  this->addFrame(new FixedRelativeFrame("B",WrOS,SqrMat(3,EYE),this->getFrame("I")));
  muffe->setFrameOfReference(this->getFrame("B"));
  muffe->setFrameForKinematics(muffe->getFrame("C"));
  muffe->setTranslation(new LinearTranslation<VecV>(JT));
  muffe->setRotation(new RotationAboutFixedAxis<VecV>(JR));
//  muffe->setRotation(new RotationAboutZAxis());

  muffe->setInitialGeneralizedPosition(spiel*Vec("[0.2;0.0;0.0]"));
  muffe->setInitialGeneralizedVelocity(      Vec("[0.0;0.0;0.0]"));

  double mu = 0.2;

  for(int i=0;i<4;i++) {
    UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
    UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
    PlanarCoulombFriction *coulFriction = new PlanarCoulombFriction(mu);
    PlanarCoulombImpact   *coulImptact  = new PlanarCoulombImpact(mu);

    stringstream name;
    name << "PM" << i;
    Point* pMuffe = new Point(name.str());
    name.clear();
    name << "KontaktMB" << i; 
    Contact *contact = new Contact(name.str());
    Vec KrSPMuffe(3);

    switch(i){
      case 0: contact->connect(pMuffe,this->getContour("LineL"));
              KrSPMuffe(0) =  R/2;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 1: contact->connect(pMuffe,this->getContour("LineL"));
              KrSPMuffe(0) =  R/2;   KrSPMuffe(1) = -hoehe/2.;
              break;
      case 2: contact->connect(pMuffe,this->getContour("LineR"));
              KrSPMuffe(0) = -R/2;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 3: contact->connect(pMuffe,this->getContour("LineR"));
              KrSPMuffe(0) = -R/2;   KrSPMuffe(1) = -hoehe/2.;
              break;
    }
    muffe->addFrame(new FixedRelativeFrame(name.str(),KrSPMuffe,SqrMat(3,EYE)));
    pMuffe->setFrameOfReference(muffe->getFrame(name.str()));
    muffe->addContour(pMuffe);
    contact->setNormalForceLaw  (cntForceLaw );
    contact->setNormalImpactLaw (impForceLaw );
    contact->setTangentialForceLaw (coulFriction);
    contact->setTangentialImpactLaw(coulImptact );
    addLink(contact);
  }

  // Drehpunkt der Feder
  Vec KrSPFederDrehpunkt(3);
  KrSPFederDrehpunkt(0) = FDPunkt;
  muffe->addFrame(new FixedRelativeFrame("Drehpunkt",KrSPFederDrehpunkt,SqrMat(3,EYE)));

  // Specht --------------------------------------
  double  mSpecht = 100.e-3;
  double  JSpecht =  0.4e-3;

  RigidBody *specht = new RigidBody("Specht");
  specht->setFrameOfReference(muffe->getFrame("Drehpunkt"));
  specht->setRotation(new RotationAboutFixedAxis<VecV>(JR));
//  specht->setRotation(new RotationAboutZAxis());
  this->addObject(specht);

  Vec MrDrehpunkt(3); /* wird von Schwerpunkt aus bemast */
  MrDrehpunkt(0) = -2.0 * FDPunkt;
  specht->addFrame(new FixedRelativeFrame("D",MrDrehpunkt,SqrMat(3,EYE),specht->getFrame("C")));
  specht->setFrameForKinematics(specht->getFrame("D"));

  Vec SrSchabelspitze(3);
  SrSchabelspitze(1) =  50.e-3;
  SrSchabelspitze(0) = 1.0 * ( - 3*FDPunkt  + r + 1.0e-3) ;

  specht->setMass(mSpecht);
  Theta(2,2) = JSpecht;
  specht->setInertiaTensor(Theta);

  Point* schnabel = new Point("Schabel");
  specht->addFrame(new FixedRelativeFrame("Schabel",SrSchabelspitze,SqrMat(3,EYE)));
  schnabel->setFrameOfReference(specht->getFrame("Schabel"));
  specht->addContour(schnabel);

  GeneralizedSpringDamper *feder = new GeneralizedSpringDamper("Drehfeder");
  feder->setRigidBody(specht);
  double cDF = 0.5;
  feder->setGeneralizedForceFunction(new LinearSpringDamperForce(cDF,0.0,0.0));
  addLink(feder);

  UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
  UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
  PlanarCoulombFriction *coulFriction = new PlanarCoulombFriction(mu);
  PlanarCoulombImpact   *coulImptact  = new PlanarCoulombImpact(mu);

  Contact *contact = new Contact("SchnabelKontakt");
  contact->connect(schnabel,this->getContour("LineL"));
  contact->setNormalForceLaw  (cntForceLaw );
  contact->setNormalImpactLaw (impForceLaw );
  contact->setTangentialForceLaw (coulFriction);
  contact->setTangentialImpactLaw(coulImptact );
  addLink(contact);

  specht->setInitialGeneralizedPosition(Vec(1,INIT, 0.0));
  specht->setInitialGeneralizedVelocity(Vec(1,INIT,-5.0));

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::IvBody* muffeMBV = new OpenMBV::IvBody;
  muffeMBV->setIvFileName("objects/muffe.wrl");
  muffeMBV->setInitialRotation( 0, 0, M_PI/2. );
  muffeMBV->setScaleFactor(R);
  muffe->setOpenMBVRigidBody(muffeMBV); 

  OpenMBV::IvBody* spechtMBV = new OpenMBV::IvBody;
  spechtMBV->setIvFileName("objects/specht.wrl");
  spechtMBV->setInitialTranslation( SrSchabelspitze(0), SrSchabelspitze(1), SrSchabelspitze(2));
  spechtMBV->setInitialRotation(M_PI/2., 0, 0);
  spechtMBV->setScaleFactor(0.05);
  specht->setOpenMBVRigidBody(spechtMBV); 
#endif
}


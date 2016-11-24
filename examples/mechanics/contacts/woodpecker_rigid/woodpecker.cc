#include "woodpecker.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contact_kinematics/point_line.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/links/generalized_spring_damper.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"

#include "openmbvcppinterface/ivbody.h"

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

  RigidBody *stange = new RigidBody("Stange");
  this->addObject(stange);

  Line *lL = new Line("LineL");
  Line *lR = new Line("LineR");

  Vec3 rS;
  rS(0)=r/2;
  rS(1)=0.5;
  stange->addFrame(new FixedRelativeFrame("LineL", rS, BasicRotAKIz(0)));
  rS(0)=-r/2;
  stange->addFrame(new FixedRelativeFrame("LineR", rS, BasicRotAKIz(M_PI)));
  lL->setFrameOfReference(stange->getFrame("LineL"));
  lR->setFrameOfReference(stange->getFrame("LineR"));
  lL->enableOpenMBV(1);
  lR->enableOpenMBV(1);
  stange->addContour(lL);
  stange->addContour(lR);

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

  muffe->setGeneralizedInitialPosition(spiel*Vec("[0.2;0.0;0.0]"));
  muffe->setGeneralizedInitialVelocity(      Vec("[0.0;0.0;0.0]"));

  double mu = 0.2;

  for(int i=0;i<4;i++) {
    UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
    UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
    PlanarCoulombFriction *coulFriction = new PlanarCoulombFriction(mu);
    PlanarCoulombImpact   *coulImptact  = new PlanarCoulombImpact(mu);

    stringstream name;
    name << "PM" << i;
    Point* pMuffe = new Point(name.str());
    pMuffe->enableOpenMBV();
    name.clear();
    name << "KontaktMB" << i; 
    Contact *contact = new Contact(name.str());
    Vec KrSPMuffe(3);

    switch(i){
      case 0: contact->connect(pMuffe,stange->getContour("LineL"));
              KrSPMuffe(0) =  R/2;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 1: contact->connect(pMuffe,stange->getContour("LineL"));
              KrSPMuffe(0) =  R/2;   KrSPMuffe(1) = -hoehe/2.;
              break;
      case 2: contact->connect(pMuffe,stange->getContour("LineR"));
              KrSPMuffe(0) = -R/2;   KrSPMuffe(1) =  hoehe/2.;
              break;
      case 3: contact->connect(pMuffe,stange->getContour("LineR"));
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
  schnabel->enableOpenMBV();
  specht->addFrame(new FixedRelativeFrame("Schabel",SrSchabelspitze,SqrMat(3,EYE)));
  schnabel->setFrameOfReference(specht->getFrame("Schabel"));
  specht->addContour(schnabel);

  GeneralizedSpringDamper *feder = new GeneralizedSpringDamper("Drehfeder");
  feder->connect(specht);
  double cDF = 0.5;
  feder->setGeneralizedForceFunction(new LinearSpringDamperForce(cDF,0.0));
  addLink(feder);

  UnilateralConstraint   *cntForceLaw = new UnilateralConstraint;
  UnilateralNewtonImpact *impForceLaw = new UnilateralNewtonImpact(0.0);
  PlanarCoulombFriction *coulFriction = new PlanarCoulombFriction(mu);
  PlanarCoulombImpact   *coulImptact  = new PlanarCoulombImpact(mu);

  Contact *contact = new Contact("SchnabelKontakt");
  contact->connect(schnabel,stange->getContour("LineL"));
  contact->setNormalForceLaw  (cntForceLaw );
  contact->setNormalImpactLaw (impForceLaw );
  contact->setTangentialForceLaw (coulFriction);
  contact->setTangentialImpactLaw(coulImptact );
  addLink(contact);

  specht->setGeneralizedInitialPosition(Vec(1,INIT, 0.0));
  specht->setGeneralizedInitialVelocity(Vec(1,INIT,-5.0));

  std::shared_ptr<OpenMBV::Frustum> stangeMBV = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  stangeMBV->setHeight(1);
  stangeMBV->setTopRadius(r/2);
  stangeMBV->setBaseRadius(r/2);
  stangeMBV->setDiffuseColor(0.,0.,170./255);
  stangeMBV->setInitialRotation(M_PI/2,0,0);
  stange->setOpenMBVRigidBody(stangeMBV);

  std::shared_ptr<OpenMBV::Frustum> muffeMBV = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  muffeMBV->setHeight(hoehe);
  muffeMBV->setTopRadius(1.2*R/2);
  muffeMBV->setBaseRadius(1.2*R/2);
  muffeMBV->setInnerTopRadius(R/2);
  muffeMBV->setInnerBaseRadius(R/2);
  muffeMBV->setDiffuseColor(100./255,1.,1.);
  muffeMBV->setInitialTranslation(0,-hoehe/2,0);
  muffeMBV->setInitialRotation(M_PI/2,0,0);
  muffe->setOpenMBVRigidBody(muffeMBV); 

  std::shared_ptr<OpenMBV::IvBody> spechtMBV = OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
  spechtMBV->setIvFileName("objects/specht.wrl");
  spechtMBV->setInitialTranslation( SrSchabelspitze(0), SrSchabelspitze(1), SrSchabelspitze(2));
  spechtMBV->setInitialRotation(M_PI/2., 0, 0);
  spechtMBV->setScaleFactor(0.05);
  specht->setOpenMBVRigidBody(spechtMBV); 
}


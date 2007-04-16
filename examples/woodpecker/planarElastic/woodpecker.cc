#include "woodpecker.h"

#include "body_rigid_abs.h"
#include "body_flexible_1s_21_rcm.h"
//#include "body_flexible_1s_21_ancf.h"
#include "contact_rigid.h"
#include "impact_rigid.h"
#include "connection_rigid.h"
#include "connection_flexible.h"

#include "objobject.h"
#include "cylinder.h"
using namespace AMVis;

Woodpecker::Woodpecker(const string &projectName) : MultiBodySystem(projectName) {
  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  setGrav(grav);

// Bewegungsrichtung der Koerper --------------
  Mat JT(3,2);
  JT(0,0) = 1;
  JT(1,1) = 1;
  Vec JR(3);
  JR(2) = 1;


// Stange -------------------------------------
  int Elements = 4;
  double     L = 1.0;

  setProjectDirectory("plot");

  BodyFlexible1s21RCM *balken = new BodyFlexible1s21RCM("Balken",true);
//  BodyFlexible1s21ANCF *balken = new BodyFlexible1s21ANCF("Balken",true);
  balken->setNumberElements(Elements); 
  Vec WrN00(3);
  WrN00(0) = 0;  WrN00(1) = 0;  WrN00(2) = 0;
  balken->setWrON00(WrN00);

  balken->setLength(L);
  balken->setEModul(7.e10);

  double h = 7.5e-3;
  double d = 7.5e-3;
  balken->setCrossSectionalArea(h*d);
  balken->setMomentInertia(h*d*h*h/12.);
//  balken->setLehrDamping(0.10);
  balken->setDensity(2.3e3);

  balken->initRelaxed(90./180.*M_PI);
  
  balken->setJT(JT);
  balken->setJR(JR);

  addObject(balken);

// In inertiale Einspannung -------------------

  balken->addPort("AnfangBalken",0);
  this->addPort("InertialEinspannung",Vec(3));
  ConnectionRigid *crF = new ConnectionRigid("Einspannung");
  crF->setForceDirection(JT);
  crF->setMomentDirection(JR);
  crF->connect(balken->getPort("AnfangBalken"),this->getPort("InertialEinspannung"));
  addLink(crF);

  SymMat Theta(3);
  Vec WrOS(3);
// Muffe -------------------------------------
  double   spiel = 0.80e-3;
  double   hoehe = 18.e-3;
  double FDPunkt = 27.e-3;
  double yMuffe0 =  0.925;
  double  mMuffe = 10.e-3;
  double  JMuffe =  5.e-6;
//
  BodyRigidAbs *muffe = new BodyRigidAbs("Muffe");
  addObject(muffe);
  WrOS(1) = yMuffe0;
  muffe->setWrOS0(WrOS);
  muffe->setJT(JT);
  muffe->setJR(JR);
  muffe->setMass(mMuffe);
  Theta(2,2) = JMuffe;
  muffe->setInertia(Theta);

// KontaktPunkte
  Point* p0muffe = new Point("PM0");
  Point* p1muffe = new Point("PM1");
  Point* p2muffe = new Point("PM2");
  Point* p3muffe = new Point("PM3");

  Vec KrSPMuffe0(3);
      KrSPMuffe0(0) =  spiel/2.;      KrSPMuffe0(1) =  hoehe/2.;
  Vec KrSPMuffe1(3);
      KrSPMuffe1(0) =  spiel/2.;      KrSPMuffe1(1) = -hoehe/2.;
  Vec KrSPMuffe2(3);
      KrSPMuffe2(0) = -spiel/2.;      KrSPMuffe2(1) =  hoehe/2.;
  Vec KrSPMuffe3(3);
      KrSPMuffe3(0) = -spiel/2.;      KrSPMuffe3(1) = -hoehe/2.;

  muffe->addContour(p0muffe,KrSPMuffe0);
  muffe->addContour(p1muffe,KrSPMuffe1);
  muffe->addContour(p2muffe,KrSPMuffe2);
  muffe->addContour(p3muffe,KrSPMuffe3);

// Kontakte Muffe-Stab
  double mu = 0.15;

  ContactRigid *cMB0 = new ContactRigid("KontaktMB0");
  ContactRigid *cMB1 = new ContactRigid("KontaktMB1");
  ContactRigid *cMB2 = new ContactRigid("KontaktMB2");
  ContactRigid *cMB3 = new ContactRigid("KontaktMB3");
//
  cMB0->connect(p0muffe,balken->getContour("L"));
  cMB0->setFrictionDirections(1);
  cMB0->setFrictionCoefficient(mu);
  cMB1->connect(p1muffe,balken->getContour("L"));
  cMB1->setFrictionDirections(1);
  cMB1->setFrictionCoefficient(mu);
  cMB2->connect(p2muffe,balken->getContour("R"));
  cMB2->setFrictionDirections(1);
  cMB2->setFrictionCoefficient(mu);
  cMB3->connect(p3muffe,balken->getContour("R"));
  cMB3->setFrictionDirections(1);
  cMB3->setFrictionCoefficient(mu);
//
  addLink(cMB0);  addLink(cMB1);  addLink(cMB2);  addLink(cMB3);
 
 cMB0->setPlotLevel(3);
 cMB1->setPlotLevel(3);
 cMB2->setPlotLevel(3);
 cMB3->setPlotLevel(3);
 
// Drehpunkt der Feder
  Vec KrSPFederDrehpunkt(3);
  KrSPFederDrehpunkt(0) = FDPunkt;
  muffe->addPort("Drehpunkt",KrSPFederDrehpunkt);


// Specht --------------------------------------
  double  mSpecht = 100.e-3;
  double  JSpecht =  0.4e-3;

  double xSpecht0 =  3*FDPunkt;
  double ySpecht0 =  yMuffe0;

  double xSchnabel =  -xSpecht0 + 2.e-3;
  double ySchnabel =   50.e-3;
//
  BodyRigidAbs *specht = new BodyRigidAbs("Specht");
  addObject(specht);

  WrOS(0) = xSpecht0; // Drehpunkt in der Mitte zwischen Muffe und Specht!?!?!?!?!
  WrOS(1) = ySpecht0;
  specht->setWrOS0(WrOS);
  specht->setJT(JT);
  specht->setJR(JR);
  specht->setMass(mSpecht);
  Theta(2,2) = JSpecht;
  specht->setInertia(Theta);

// Drehpunkt der Feder
  KrSPFederDrehpunkt(0) = -2*FDPunkt;
  specht->addPort("Drehpunkt",KrSPFederDrehpunkt);


  Mat FD(3,2,INIT,0.0);
  FD(0,0) = 1.0;  FD(1,1) = 1.0;//  FD(2,2) = 1.0;
  Vec MD(3,INIT,0.0);
  MD(2) = 1.0;
// Drehfeder ----------------------------------
// "Achse"
  ConnectionRigid *achseT = new ConnectionRigid("AchseTrans");
//  ConnectionFlexibleLinear *achseT = new ConnectionFlexibleLinear(this,"AchseTrans");
  achseT->connect(specht->getPort("Drehpunkt"),muffe->getPort("Drehpunkt"));
//  achseT->setStiffness(1e3);
//  achseT->setDamping(1e1);
  achseT->setForceDirection(FD);
  addLink(achseT);
// Feder
  double cDF = 0.5;
//
  ConnectionFlexible *feder = new ConnectionFlexible("Drehfeder");
  feder->connect(specht->getPort("Drehpunkt"),muffe->getPort("Drehpunkt"));
  feder->setMomentDirection(MD);
  feder->setRotationalStiffness(cDF);
  feder->setPlotLevel(3);
  addLink(feder);


   Point* schnabel = new Point("Schabel");

  Vec KrSPSchabel(3);
      KrSPSchabel(0) =  xSchnabel;
      KrSPSchabel(1) =  ySchnabel;
  specht->addContour(schnabel,KrSPSchabel);
  

// Kontakt Schnabel - Stab
  double eps = 0.7/(1.0+0.7);
//
  ImpactRigid *cSchnabel = new ImpactRigid("SchnabelKontakt");
  // ContactRigid *cSchnabel = new ContactRigid(this, "SchnabelKontakt");
  // ContactRigidFrictionless *cSchnabel = new ContactRigidFrictionless(this, "SchnabelKontakt");
//
  cSchnabel->connect(schnabel,balken->getContour("L"));
  cSchnabel->setNormalRestitutionCoefficient(eps);
  cSchnabel->setFrictionDirections(1);
  cSchnabel->setFrictionCoefficient(mu);
  addLink(cSchnabel);
 
  Vec u0(3);
  u0(2) = -5;
  specht->setu0(u0);
 
  specht->setPlotLevel(2);
  balken->setPlotLevel(2);
  muffe ->setPlotLevel(2);

  balken->createAMVisBody();
//   balken->createAMVisBody();

  ObjObject *spechtobj = new ObjObject(specht->getFullName(),1,true);
  spechtobj->setColor(0);
  spechtobj->setObjFilename("objects/specht.obj");
  spechtobj->setInitialTranslation(-0.01, 0.01, 0);
  spechtobj->setInitialRotation(-1.570796, 0, 0);
  spechtobj->setScaleFactor(0.04);
  specht->setAMVisBody(spechtobj);  

  Cylinder* cylinder = new Cylinder(muffe->getFullName(),1,true);
  cylinder->setHeight(hoehe);
  cylinder->setInitialRotation(M_PI/2.,0,0);
  cylinder->setBaseRadius(0.01);
  cylinder->setTopRadius (0.01);
  cylinder->setColor(1.0);
  muffe->setAMVisBody(cylinder);
}


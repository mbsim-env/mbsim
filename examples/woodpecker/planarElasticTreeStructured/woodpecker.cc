#include "woodpecker.h"
#include "body_rigid_rel.h"
#include "body_flexible_1s_21_rcm.h"
#include "userfunction.h"
#include "load.h"
#include "port.h"
#include "contour.h"
#include "contact_flexible.h"
#include "contact_rigid.h"
#include "impact_rigid.h"
#include "connection_rigid.h"
#include "connection_flexible.h"
#include "spring_rotational_rel.h"

#include "cuboid.h"
#include "cylinder.h"
#include "objobject.h"
using namespace AMVis;


Woodpecker::Woodpecker(const string &projectName) : MultiBodySystem(projectName) {
  setProjectDirectory("plot");

  // Gravitation
  Vec grav(3,INIT,0.0);
  grav(1)=-9.81;
  setGrav(grav);
  double     r = 10.e-3;

  // Bewegungsrichtung der Koerper --------------
  Mat JT(3,2);
  JT(0,0) = 1;
  JT(1,1) = 1;
  Vec JR(3);
  JR(2) = 1;


  // Stange -------------------------------------
  int Elements = 5;
  double     L = 1.0;

  //  BodyFlexible1s21ANCF *balken = new BodyFlexible1s21ANCF(this,"Balken",true);
  BodyFlexible1s21RCM *balken = new BodyFlexible1s21RCM("Balken",true);
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
  balken->setLehrDamping(0.10);
  balken->setDensity(2.3e3);

  balken->initRelaxed(90./180.*M_PI);

  balken->setJT(JT);
  balken->setJR(JR);

  addObject(balken);
  double spiel = 0.7e-3;

  double       R =  r + spiel/2.;

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
  double   hoehe = 18.e-3;
  double FDPunkt = 27.e-3;
  double yMuffe0 =  0.925;
  double  mMuffe = 10.e-3;
  double  JMuffe =  5.e-6;
  // 

  Tree *tree = new Tree("Baum"); 
  addObject(tree);


  BodyRigidRel *muffe = new BodyRigidRel("Muffe");
  tree->setRoot(muffe);
  WrOS(1) = yMuffe0;
  muffe->setPrPK0(WrOS);
  muffe->setJT(JT);
  muffe->setJR(JR);
  muffe->setMass(mMuffe);
  Theta(2,2) = JMuffe;
  muffe->setInertia(Theta,true);

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
  //
  ContactRigid *cMB0 = new ContactRigid("KontaktMB0");
  cMB0->setFrictionDirections(1);
  cMB0 -> setPlotLevel(2);
  ContactRigid *cMB1 = new ContactRigid("KontaktMB1");
  cMB1->setFrictionDirections(1);
  cMB1 -> setPlotLevel(2);
  ContactRigid *cMB2 = new ContactRigid("KontaktMB2");
  cMB2->setFrictionDirections(1);
  ContactRigid *cMB3 = new ContactRigid("KontaktMB3");
  cMB3->setFrictionDirections(1);
  //
  cMB0->connect(p0muffe,balken->getContour("L"));
  cMB0->setFrictionCoefficient(mu);
  cMB1->connect(p1muffe,balken->getContour("L"));
  cMB1->setFrictionCoefficient(mu);
  cMB2->connect(p2muffe,balken->getContour("R"));
  cMB2->setFrictionCoefficient(mu);
  cMB3->connect(p3muffe,balken->getContour("R"));
  cMB3->setFrictionCoefficient(mu);
  //
  addLink(cMB0);  addLink(cMB1);  addLink(cMB2);  addLink(cMB3);

  // Drehpunkt der Feder
  Vec KrSPFederDrehpunkt(3);
  KrSPFederDrehpunkt(0) = FDPunkt;
  muffe->addPort("Drehpunkt",KrSPFederDrehpunkt);

  // Specht --------------------------------------
  double  mSpecht = 100.e-3;
  double  JSpecht =  0.4e-3;

  BodyRigidRel *specht = new BodyRigidRel("Specht");
  muffe->addChild(specht);
  Vec MrDrehpunkt(3);
  double drehpunkt =  18.e-3;
  MrDrehpunkt(0) = 1.0 * drehpunkt;
  Vec SrSchwerpunkt(3);
  SrSchwerpunkt(0) = 1.0 * 2 * drehpunkt;
  Vec SrSchabelspitze(3);
  SrSchabelspitze(1) =  50.e-3;
  SrSchabelspitze(0) = 1.0 * ( - 3*drehpunkt + 2.e-3 + r -4e-3) ;

  specht->setPrPK0(MrDrehpunkt);
  specht->setKrKS(SrSchwerpunkt);

  specht->setJR(JR);
  specht->setMass(mSpecht);
  Theta(2,2) = JSpecht;
  specht->setInertia(Theta,true);



  Point* schnabel = new Point("Schabel");
  specht->addContour(schnabel,SrSchabelspitze+SrSchwerpunkt);

  SpringRotationalRel *feder = new SpringRotationalRel("Drehfeder");
  muffe ->addPort("COG",Vec(3));
  specht->addPort("COG",SrSchwerpunkt);//Vec(3));
  feder->connect(muffe->getPort("COG"),specht->getPort("COG"));
  feder->setMomentDirection(JR);
  double cDF = 0.5;
  feder->setStiffness(cDF);
  addLink(feder);

  double eps = 0;
  ImpactRigid *cSchnabel = new ImpactRigid("SchnabelKontakt");
  cSchnabel->connect(schnabel,balken->getContour("L"));
  cSchnabel->setFrictionDirections(1);
  cSchnabel->setNormalRestitutionCoefficient(eps);
  cSchnabel->setFrictionCoefficient(mu);
  addLink(cSchnabel);

  specht->setu0(Vec(1,INIT,-5.0));


  specht->setPlotLevel(2);
  balken->setPlotLevel(2);
  muffe ->setPlotLevel(2);

  balken->createAMVisBody(false);

  ObjObject *spechtobj = new ObjObject(specht->getFullName(),1,true);
  spechtobj->setColor(0);
  spechtobj->setObjFilename("objects/specht.obj");
  spechtobj->setInitialTranslation(0.02, 0.03, 0.0);// SrSchabelspitze(2));
  spechtobj->setInitialRotation(-M_PI/2., 0, 0);
  spechtobj->setScaleFactor(0.036);
  specht->setAMVisBody(spechtobj);  

  Cylinder* cylinder = new Cylinder(muffe->getFullName(),1,true);
  cylinder->setHeight(hoehe);
  cylinder->setInitialTranslation(0, hoehe/2, 0);// SrSchabelspitze(2));
  cylinder->setInitialRotation(M_PI/2.,0,0);
  cylinder->setBaseRadius(R*1.1);
  cylinder->setTopRadius (R*1.1);
  cylinder->setColor(1.0);
  muffe->setAMVisBody(cylinder);
}


#include "system.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsim/rigid_body.h"
#include "mbsim/multi_contact.h"
#include "mbsim/environment.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/circle.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSimFlexibleBody;
using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  /* Allgemeine Parameter */
  Vec grav(3,INIT,0.); 
  grav(2) = 9.81; 
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* Parameter Lager A */
  double StiffnessLagerA = 1e6;
  double DampingLagerA = 10;

  /* Parameter Lager B */
  double StiffnessLagerB = 6e6;
  double DampingLagerB = 10;

  // Parameter Gleitlager
  double m_GL = 2.3;              	    // Masse Gleitlager
  SymMat ThetaGL(3,INIT,0.);
  ThetaGL(0,0) = 0.007;                 // ACHTUNG DUMMYWERT
  ThetaGL(1,1) = 1.e-4;                 // ACHTUNG DUMMYWERT
  ThetaGL(2,2) = 1.e-4;                 // ACHTUNG DUMMYWERT
  double R_GL = 1.01e-2; 			    // Gleitlagerinnendurchmesser    
  double mu = 0.01;               	    // Reibkoeffizient

  /* Parameter Welle */
  int Elements = 5;               	    // Anzahl der Elemente der flex. Welle
  double L = 0.59;                	    // Länge
  double r = 12.5e-3;             	    // Radius
  double rho = 7.85e3;            	    // Dichte
  double E = 2.1e11;              	    // E-Modul
  double G = 0.81e11;             	    // Schubmodul

  /* Parameter Schwungrad  */ 
  double PosScheibeS = 1.8e-1;   	    // Position Schwungrad auf Welle
  double mScheibeS = 4.6+0.38;
  SymMat ThetaScheibeS(3,INIT,0.);
  ThetaScheibeS(0,0) = 0.01;
  ThetaScheibeS(1,1) = 0.003;           // ACHTUNG DUMMYWERT
  ThetaScheibeS(2,2) = 0.003;           // ACHTUNG DUMMYWERT

  /* Parameter Scheibe Gleitlager */
  double mScheibeGLS = 0.001;
  SymMat ThetaScheibeGLS(3,INIT,0.);
  ThetaScheibeGLS(0,0) = 1.e-6;         // ACHTUNG DUMMYWERT
  ThetaScheibeGLS(1,1) = 1.e-6;         // ACHTUNG DUMMYWERT
  ThetaScheibeGLS(2,2) = 1.e-6;         // ACHTUNG DUMMYWERT
  double R_GLS = 1e-2;       		    // Durchmesser Scheibe Gleitlager
  double b_GLS = 19.e-3;           	    // Halbe Breite Scheibe Gleitlager

  /* Definition MKS */
  /* Welle */
  FlexibleBody1s23BTA *welle = new FlexibleBody1s23BTA("Rotorwelle");	
  double A = M_PI*r*r;                  // Querschnitt 
  double IB = M_PI*r*r*r*r/4.;          // Biegesteifigkeit	
  welle->setNumberElements(Elements);   //Anz. der Elemente der elast. Welle   
  welle->setLength(L);
  welle->setElastModuls(E,G);
  welle->setCrossSectionalArea(A);
  welle->setMomentsInertia(IB,IB,2*IB);
  welle->setDensity(rho);
  welle->setContourRadius(R_GLS/2.);
  welle->setFrameOfReference(this->getFrame("I"));
  welle->setq0(Vec(5*Elements+5,INIT,0.));	
  welle->addFrame("Anfang",0.);
  welle->addFrame("Ende",L);
  welle->addFrame("PosScheibeS",PosScheibeS);
  welle->setMassProportionalDamping(60);
  welle->setTorsionalDamping(0.01);
  this->addObject(welle);
  #ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cylinder=new OpenMBV::SpineExtrusion;
  cylinder->setNumberOfSpinePoints(Elements*4+1); // resolution of visualisation
  cylinder->setStaticColor(0.6); // color in (minimalColorValue, maximalColorValue)
  cylinder->setScaleFactor(1.); // orthotropic scaling of cross section
  vector<OpenMBV::PolygonPoint*> *circle = new vector<OpenMBV::PolygonPoint*>; // clockwise ordering, no doubling for closure
  for(int i=0;i<20;i++) {
    OpenMBV::PolygonPoint *corner  = new OpenMBV::PolygonPoint(R_GLS*0.5*cos(i*2*M_PI/20),R_GLS*0.5*sin(i*2*M_PI/20),1);
    circle->push_back(corner);
  }
  cylinder->setContour(circle);
  welle->setOpenMBVSpineExtrusion(cylinder);
#endif

  /* Schwungrad */
  RigidBody *ScheibeS = new RigidBody("Schwungrad");
  Vec Wr0_S(3,INIT,0.); Wr0_S(0) = PosScheibeS;
  this->addFrame("ScheibeS",Wr0_S,SqrMat(3,EYE));
  ScheibeS->setFrameForKinematics(ScheibeS->getFrame("C"));
  ScheibeS->setFrameOfReference(this->getFrame("ScheibeS")); 
  ScheibeS->setTranslation(new LinearTranslation(Mat(3,3,EYE)));
  ScheibeS->setRotation(new CardanAngles());
  ScheibeS->setMass(mScheibeS);
  ScheibeS->setInertiaTensor(ThetaScheibeS);
  this->addObject(ScheibeS);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum* ScheibeSMBV = new OpenMBV::Frustum;
  ScheibeSMBV->setBaseRadius(2.*r);
  ScheibeSMBV->setTopRadius(2.*r);
  ScheibeSMBV->setHeight(L/20.); 
  ScheibeSMBV->setScaleFactor(1.);
  ScheibeSMBV->setMinimalColorValue(0);
  ScheibeSMBV->setMaximalColorValue(1);
  ScheibeSMBV->setStaticColor(0);
  ScheibeSMBV->setInitialTranslation(L/40.,0.,0.);
  ScheibeSMBV->setInitialRotation(0.,M_PI/2.,0.);
  ScheibeS->setOpenMBVRigidBody(ScheibeSMBV);
#endif

  /* Gleitlager Scheibe */
  RigidBody *ScheibeGLS = new RigidBody("Gleitlagerscheibe"); 
  ScheibeGLS->setFrameForKinematics(ScheibeGLS->getFrame("C"));
  ScheibeGLS->setFrameOfReference(this->getFrame("I")); 
  ScheibeGLS->setTranslation(new LinearTranslation(Mat(3,3,EYE)));
  ScheibeGLS->setRotation(new CardanAngles());
  ScheibeGLS->setMass(mScheibeGLS);
  ScheibeGLS->setInertiaTensor(ThetaScheibeGLS);

  Frustum* cylsurf_GLS = new Frustum("cylsurf_GLS");
  cylsurf_GLS->setRadii(Vec(2,INIT,R_GLS));
  cylsurf_GLS->setHeight(2.*b_GLS);
  cylsurf_GLS->setOutCont(true);
#ifdef HAVE_OPENMBVCPPINTERFACE
  cylsurf_GLS->enableOpenMBV();
#endif
  SqrMat AWK_cylsurf_GLS(3,INIT,0.); AWK_cylsurf_GLS(2,2) = 1.; AWK_cylsurf_GLS(0,0) = cos(M_PI/2.); AWK_cylsurf_GLS(1,1) = cos(M_PI/2.); AWK_cylsurf_GLS(0,1) = sin(M_PI/2.); AWK_cylsurf_GLS(1,0) = -sin(M_PI/2.);
  Vec KrKS_cylsurf_GLS(3,INIT,0.); KrKS_cylsurf_GLS(0) = -b_GLS;
  ScheibeGLS->addContour(cylsurf_GLS,KrKS_cylsurf_GLS,AWK_cylsurf_GLS);
  this->addObject(ScheibeGLS);

  /* Gleitlager */
  RigidBody *Gleitlager = new RigidBody("Gleitlager");
  Gleitlager->setFrameForKinematics(Gleitlager->getFrame("C")); 
  Gleitlager->setFrameOfReference(this->getFrame("I")); 
  Gleitlager->setTranslation(new LinearTranslation(Mat(3,3,EYE)));
  Gleitlager->setRotation(new CardanAngles());
  Gleitlager->setMass(m_GL);
  Gleitlager->setInertiaTensor(ThetaGL);

  Circle* rightCircle = new Circle("rightCircle");
  rightCircle->setRadius(R_GL);
  rightCircle->setOutCont(false);
#ifdef HAVE_OPENMBVCPPINTERFACE
  rightCircle->enableOpenMBV();
#endif
  SqrMat AWK_rightCircle(3,INIT,0.); AWK_rightCircle(1,1) = 1.; AWK_rightCircle(0,0) = cos(M_PI/2.); AWK_rightCircle(2,2) = cos(M_PI/2.); AWK_rightCircle(0,2) = sin(M_PI/2.); AWK_rightCircle(2,0) = -sin(M_PI/2.);
  Vec PosRightCircle(3,INIT,0.); PosRightCircle(0) = b_GLS/2.;
  Gleitlager->addContour(rightCircle,PosRightCircle,AWK_rightCircle);
  this->addObject(Gleitlager);

  /* Lager A */
  Vec VTemp(3,INIT,0.);
  this->addFrame("Lager_A_Frame",VTemp,SqrMat(3,EYE)); 
  Joint *alager = new Joint("Lager_A");
  alager->setForceDirection(Mat(3,3,EYE));
  alager->setMomentDirection(Mat(3,3,EYE));
  alager->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerA,DampingLagerA)));
  alager->setMomentLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerA,DampingLagerA)));
  alager->connect(this->getFrame("Lager_A_Frame"),Gleitlager->getFrame("C"));
  this->addLink(alager);

  /* Lager B */
  VTemp(0) = L;
  this->addFrame("Lager_B_Frame",VTemp,SqrMat(3,EYE));
  Joint *blager = new Joint("Lager_B");
  blager->setForceDirection(Mat(3,3,EYE));
  blager->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerB,DampingLagerB)));
  blager->connect(this->getFrame("Lager_B_Frame"),welle->getFrame("Ende"));
  this->addLink(blager);

  /* Verbindung Schwungrad - Welle */
  Joint *VerbScheibeS = new Joint("Verbindung_Schwungrad_Welle");
  VerbScheibeS->setForceDirection(Mat(3,3,EYE));
  VerbScheibeS->setMomentDirection(Mat(3,3,EYE));
  VerbScheibeS->setForceLaw(new BilateralConstraint());
  VerbScheibeS->setMomentLaw(new BilateralConstraint());
  VerbScheibeS->setImpactForceLaw(new BilateralImpact());
  VerbScheibeS->setImpactMomentLaw(new BilateralImpact());
  VerbScheibeS->connect(ScheibeS->getFrame("C"),welle->getFrame("PosScheibeS"));
  this->addLink(VerbScheibeS);

  /* Verbindung Scheibe Gleitlager - Welle */
  Joint *VerbScheibeGLS = new Joint("Verbindung_ScheibeGLS_Welle");
  VerbScheibeGLS->setForceDirection(Mat(3,3,EYE));
  VerbScheibeGLS->setMomentDirection(Mat(3,3,EYE));
  VerbScheibeGLS->setForceLaw(new BilateralConstraint());
  VerbScheibeGLS->setMomentLaw(new BilateralConstraint());
  VerbScheibeGLS->setImpactForceLaw(new BilateralImpact());
  VerbScheibeGLS->setImpactMomentLaw(new BilateralImpact());
  VerbScheibeGLS->connect(ScheibeGLS->getFrame("C"),welle->getFrame("Anfang"));
  this->addLink(VerbScheibeGLS);

  /* Kontakte im Gleitlager */
  Contact *cGL_Right = new Contact("KontaktGL0");
  cGL_Right->connect(Gleitlager->getContour("rightCircle"),ScheibeGLS->getContour("cylsurf_GLS"));
  cGL_Right->setContactForceLaw(new UnilateralConstraint);
  cGL_Right->setContactImpactLaw(new UnilateralNewtonImpact);
  cGL_Right->setFrictionForceLaw(new SpatialCoulombFriction(mu));
  cGL_Right->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  this->addLink(cGL_Right);  
}


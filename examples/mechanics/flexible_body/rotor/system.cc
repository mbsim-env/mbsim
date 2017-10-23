#include "system.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsimFlexibleBody/frames/frame_1s.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/contact.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/constant_function.h"
#include "mbsim/functions/step_function.h"
#include "mbsim/observers/contact_observer.h"

#include "openmbvcppinterface/frustum.h"

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

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
  double AntriebsmomentLagerB = 1e2; // wird nach 0.05s auf 0 gesetzt
  double SchlagLagerB = 1e1;

  // Parameter Gleitlager
  double m_GL = 2.3;              	    // Masse Gleitlager
  SymMat ThetaGL(3,INIT,0.);
  ThetaGL(0,0) = 0.007;                 // ACHTUNG DUMMYWERT
  ThetaGL(1,1) = 1.e-4;                 // ACHTUNG DUMMYWERT
  ThetaGL(2,2) = 1.e-4;                 // ACHTUNG DUMMYWERT
  double R_GL = 1.01e-2; 			          // Gleitlagerinnendurchmesser    
  double mu = 0.01;               	    // Reibkoeffizient

  /* Parameter Welle */
  int Elements = 5;               	    // Anzahl der Elemente der flex. Welle
  double L = 0.59;                	    // Länge
  double r = 12.5e-3;             	    // Radius
  double rho = 7.85e3;            	    // Dichte
  double E = 2.1e11;              	    // E-Modul
  double G = 0.81e11;             	    // Schubmodul

  /* Parameter Schwungrad  */ 
  double PosScheibeS = 1.8e-1;   	      // Position Schwungrad auf Welle
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
  double R_GLS = 1e-2;       		        // Durchmesser Scheibe Gleitlager
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
//  welle->setContourRadius(R_GLS/2.);
  welle->setFrameOfReference(this->getFrame("I"));
  welle->setq0(Vec(5*Elements+5,INIT,0.));	
  welle->addFrame(new Frame1s("Anfang",0.));
  welle->addFrame(new Frame1s("Ende",L));
  welle->addFrame(new Frame1s("PosScheibeS",PosScheibeS));
  welle->getFrame("Anfang")->enableOpenMBV(0.01);
  welle->getFrame("Ende")->enableOpenMBV(0.01);
  welle->getFrame("PosScheibeS")->enableOpenMBV(0.01);
  welle->setMassProportionalDamping(60);
  welle->setTorsionalDamping(0.01);
  this->addObject(welle);
  std::shared_ptr<OpenMBV::SpineExtrusion> cylinder=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cylinder->setNumberOfSpinePoints(Elements*4+1); // resolution of visualisation
  cylinder->setDiffuseColor(0.26667, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cylinder->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > circle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  for(int i=0;i<20;i++) {
    shared_ptr<OpenMBV::PolygonPoint>  corner  = OpenMBV::PolygonPoint::create(R_GLS*0.5*cos(i*2*M_PI/20),R_GLS*0.5*sin(i*2*M_PI/20),1);
    circle->push_back(corner);
  }
  cylinder->setContour(circle);
  welle->setOpenMBVSpineExtrusion(cylinder);

  /* Schwungrad */
  RigidBody *ScheibeS = new RigidBody("Schwungrad");
  Vec Wr0_S(3,INIT,0.); Wr0_S(0) = PosScheibeS; 
  this->addFrame(new FixedRelativeFrame("ScheibeS",Wr0_S,SqrMat(3,EYE)));
  ScheibeS->setFrameForKinematics(ScheibeS->getFrame("C"));
  ScheibeS->setFrameOfReference(this->getFrame("ScheibeS")); 
  ScheibeS->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  ScheibeS->setRotation(new RotationAboutAxesXYZ<VecV>);
  ScheibeS->setMass(mScheibeS);
  ScheibeS->setInertiaTensor(ThetaScheibeS);
  this->addObject(ScheibeS);

  std::shared_ptr<OpenMBV::Frustum> ScheibeSMBV = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  ScheibeSMBV->setBaseRadius(2.*r);
  ScheibeSMBV->setTopRadius(2.*r);
  ScheibeSMBV->setHeight(L/20.); 
  ScheibeSMBV->setScaleFactor(1.);
  ScheibeSMBV->setMinimalColorValue(0);
  ScheibeSMBV->setMaximalColorValue(1);
  ScheibeSMBV->setDiffuseColor(2/3.0, 1, 1);
  ScheibeSMBV->setInitialTranslation(L/40.,0.,0.);
  ScheibeSMBV->setInitialRotation(0.,M_PI/2.,0.);
  ScheibeS->setOpenMBVRigidBody(ScheibeSMBV);

  /* Gleitlager Scheibe */
  RigidBody *ScheibeGLS = new RigidBody("Gleitlagerscheibe"); 
  ScheibeGLS->setFrameForKinematics(ScheibeGLS->getFrame("C"));
  ScheibeGLS->setFrameOfReference(this->getFrame("I")); 
  ScheibeGLS->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  ScheibeGLS->setRotation(new RotationAboutAxesXYZ<VecV>);
  ScheibeGLS->setMass(mScheibeGLS);
  ScheibeGLS->setInertiaTensor(ThetaScheibeGLS);

  Frustum* cylsurf_GLS = new Frustum("cylsurf_GLS");
  cylsurf_GLS->setRadii(Vec(2,INIT,R_GLS));
  cylsurf_GLS->setHeight(2.*b_GLS);
  cylsurf_GLS->setOutCont(true);
  cylsurf_GLS->enableOpenMBV();
  SqrMat AWK_cylsurf_GLS(3,INIT,0.); AWK_cylsurf_GLS(2,2) = 1.; AWK_cylsurf_GLS(0,0) = cos(M_PI/2.); AWK_cylsurf_GLS(1,1) = cos(M_PI/2.); AWK_cylsurf_GLS(0,1) = sin(M_PI/2.); AWK_cylsurf_GLS(1,0) = -sin(M_PI/2.);
  Vec KrKS_cylsurf_GLS(3,INIT,0.); KrKS_cylsurf_GLS(0) = -b_GLS;
  ScheibeGLS->addFrame(new FixedRelativeFrame("P",KrKS_cylsurf_GLS,AWK_cylsurf_GLS));
  cylsurf_GLS->setFrameOfReference(ScheibeGLS->getFrame("P"));
  ScheibeGLS->addContour(cylsurf_GLS);
  this->addObject(ScheibeGLS);

  /* Gleitlager */
  RigidBody *Gleitlager = new RigidBody("Gleitlager");
  Gleitlager->setFrameForKinematics(Gleitlager->getFrame("C")); 
  Gleitlager->setFrameOfReference(this->getFrame("I")); 
  Gleitlager->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  Gleitlager->setRotation(new RotationAboutAxesXYZ<VecV>);
  Gleitlager->setMass(m_GL);
  Gleitlager->setInertiaTensor(ThetaGL);

  Circle* rightCircle = new Circle("rightCircle");
  rightCircle->setRadius(R_GL);
  rightCircle->setSolid(false);
  rightCircle->enableOpenMBV();
  SqrMat AWK_rightCircle(3,INIT,0.); AWK_rightCircle(1,1) = 1.; AWK_rightCircle(0,0) = cos(M_PI/2.); AWK_rightCircle(2,2) = cos(M_PI/2.); AWK_rightCircle(0,2) = sin(M_PI/2.); AWK_rightCircle(2,0) = -sin(M_PI/2.);
  Vec PosRightCircle(3,INIT,0.); PosRightCircle(0) = b_GLS/2.;
  Gleitlager->addFrame(new FixedRelativeFrame("R",PosRightCircle,AWK_rightCircle));
  rightCircle->setFrameOfReference(Gleitlager->getFrame("R"));
  Gleitlager->addContour(rightCircle);
  this->addObject(Gleitlager);

  /* Lager A */
  Vec VTemp(3,INIT,0.);
  this->addFrame(new FixedRelativeFrame("Lager_A_Frame",VTemp,SqrMat(3,EYE))); 
  Joint *alager = new Joint("Lager_A");
  alager->setForceDirection(Mat(3,3,EYE));
  alager->setMomentDirection(Mat(3,3,EYE));
  alager->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerA,DampingLagerA)));
  alager->setMomentLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerA,DampingLagerA)));
  alager->connect(this->getFrame("Lager_A_Frame"),Gleitlager->getFrame("C"));
  this->addLink(alager);

  /* Lager B */
  VTemp(0) = L;
  this->addFrame(new FixedRelativeFrame("Lager_B_Frame",VTemp,SqrMat(3,EYE)));
  Joint *blager = new Joint("Lager_B");
  blager->setForceDirection(Mat(3,3,EYE));
  blager->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerB,DampingLagerB)));
  blager->connect(this->getFrame("Lager_B_Frame"),welle->getFrame("Ende"));
  this->addLink(blager);

  /* Antrieb am Lager B */
  KineticExcitation *bantrieb = new KineticExcitation("Lager_B_Antrieb");
  bantrieb->setMomentFunction(new StepFunction<VecV(double)>(0.05,0.,AntriebsmomentLagerB));
  bantrieb->setMomentDirection("[1;0;0]");
  bantrieb->connect(welle->getFrame("Ende"));
  bantrieb->enableOpenMBV(0.001);
  this->addLink(bantrieb);

  KineticExcitation *bSchlag = new KineticExcitation("Lager_B_Schlag");
  bSchlag->setMomentFunction(new ConstantFunction<VecV(double)>(SchlagLagerB));
  bSchlag->setMomentDirection("[0;0;-1]");
  bSchlag->connect(welle->getFrame("Ende"));
  bSchlag->enableOpenMBV(0.001);
  this->addLink(bSchlag);

  /* Verbindung Schwungrad - Welle */
  Joint *VerbScheibeS = new Joint("Verbindung_Schwungrad_Welle");
  VerbScheibeS->setForceDirection(Mat(3,3,EYE));
  VerbScheibeS->setMomentDirection(Mat(3,3,EYE));
  VerbScheibeS->setForceLaw(new BilateralConstraint());
  VerbScheibeS->setMomentLaw(new BilateralConstraint());
  VerbScheibeS->connect(ScheibeS->getFrame("C"),welle->getFrame("PosScheibeS"));
  this->addLink(VerbScheibeS);

  /* Verbindung Scheibe Gleitlagerscheibe - Welle */
  Joint *VerbScheibeGLS = new Joint("Verbindung_ScheibeGLS_Welle");
  VerbScheibeGLS->setForceDirection(Mat(3,3,EYE));
  VerbScheibeGLS->setMomentDirection(Mat(3,3,EYE));
  VerbScheibeGLS->setForceLaw(new BilateralConstraint());
  VerbScheibeGLS->setMomentLaw(new BilateralConstraint());
  VerbScheibeGLS->connect(ScheibeGLS->getFrame("C"),welle->getFrame("Anfang"));
  this->addLink(VerbScheibeGLS);

  /* Kontakte im Gleitlager */
  Contact *cGL_Right = new Contact("KontaktGL0");
  cGL_Right->connect(Gleitlager->getContour("rightCircle"),ScheibeGLS->getContour("cylsurf_GLS"));
  cGL_Right->setNormalForceLaw(new UnilateralConstraint);
  cGL_Right->setNormalImpactLaw(new UnilateralNewtonImpact);
  cGL_Right->setTangentialForceLaw(new SpatialCoulombFriction(mu));
  cGL_Right->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  this->addLink(cGL_Right);  

  ContactObserver *observer = new ContactObserver(cGL_Right->getName()+"_Observer");
  addObserver(observer);
  observer->setContact(cGL_Right);
  observer->enableOpenMBVContactPoints(0.01);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}


#include "flexible_rotor_ehd.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_23_bta.h"
#include "mbsim/rigid_body.h"

#include "mbsim/joint.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/circle.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"
#include "mbsim/functions/basic_functions.h"

#include "mbsimEHD/contact_kinematics/cylindersolid_cylinderhollow_ehd.h"
#include "mbsimEHD/ehd_contact.h"
#include "mbsimEHD/ehd_mesh.h"
#include "cylinderflexiblesolid_cylinderhollow_ehd.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace MBSimEHD;
using namespace fmatvec;
using namespace std;

FlexibleRotorEHD::FlexibleRotorEHD(const string &projectName) :
    DynamicSystemSolver(projectName) {

  /* Allgemeine Parameter */
  Vec grav(3, INIT, 0.);
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
  double m_GL = 2.3;                    // Masse Gleitlager
  SymMat ThetaGL(3, INIT, 0.);
  ThetaGL(0, 0) = 0.007;                 // ACHTUNG DUMMYWERT
  ThetaGL(1, 1) = 1.e-4;                 // ACHTUNG DUMMYWERT
  ThetaGL(2, 2) = 1.e-4;                 // ACHTUNG DUMMYWERT
  double mu = 0.01;                      // Reibkoeffizient

  /* Parameter Welle */
  int Elements = 5;                     // Anzahl der Elemente der flex. Welle
  double L = 0.59;                      // Länge
  double radius = 12.5e-3;              // Radius
  double rho = 7.85e3;                  // Dichte
  double E = 2.1e11;                    // E-Modul
  double G = 0.81e11;                   // Schubmodul

  /* Parameter Scheibe Gleitlager */
  double mScheibeGLS = 0.001;
  SymMat ThetaScheibeGLS(3, INIT, 0.);
  ThetaScheibeGLS(0, 0) = 1.e-6;         // ACHTUNG DUMMYWERT
  ThetaScheibeGLS(1, 1) = 1.e-6;         // ACHTUNG DUMMYWERT
  ThetaScheibeGLS(2, 2) = 1.e-6;         // ACHTUNG DUMMYWERT
  double clearance = 1e-4;               // Spaltbreite
  double R_GLS = radius + clearance;     // Radius Scheibe Gleitlager
  double b_GLS = 19.e-3;                 // Halbe Breite Scheibe Gleitlager

  /* Definition MKS */
  /* Welle */
  FlexibleBody1s23BTA *welle = new FlexibleBody1s23BTA("Rotorwelle");
  double A = M_PI * radius * radius;               // Querschnitt
  double IB = M_PI * radius * radius * radius * radius / 4.; // Biegesteifigkeit
  welle->setNumberElements(Elements);    //Anz. der Elemente der elast. Welle
  welle->setLength(L);
  welle->setElastModuls(E, G);
  welle->setCrossSectionalArea(A);
  welle->setMomentsInertia(IB, IB, 2 * IB);
  welle->setDensity(rho);
  welle->setContourRadius(radius);
  welle->setFrameOfReference(getFrameI());
  welle->setq0(Vec(5 * Elements + 5, INIT, 0.));
  welle->addFrame("Anfang", 0.);
  welle->addFrame("Ende", L);
  welle->setMassProportionalDamping(60);
  welle->setTorsionalDamping(0.01);
  this->addObject(welle);
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::SpineExtrusion> cylinder=OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cylinder->setNumberOfSpinePoints(Elements*4+1); // resolution of visualisation
  cylinder->setDiffuseColor(1, 0, 0.5);// color in (minimalColorValue, maximalColorValue)
  cylinder->setScaleFactor(1.);// orthotropic scaling of cross section
  boost::shared_ptr<vector<boost::shared_ptr<OpenMBV::PolygonPoint> > > circle = boost::make_shared<vector<boost::shared_ptr<OpenMBV::PolygonPoint> > >();// clockwise ordering, no doubling for closure
  for(int i=0;i<20;i++) {
    boost::shared_ptr<OpenMBV::PolygonPoint> corner = OpenMBV::PolygonPoint::create(radius*cos(i*2*M_PI/20),radius*sin(i*2*M_PI/20),1);
    circle->push_back(corner);
  }
  cylinder->setContour(circle);
  welle->setOpenMBVSpineExtrusion(cylinder);
#endif

  /* Gleitlager Scheibe */
  RigidBody *Gleitlager = new RigidBody("Gleitlager");
  Gleitlager->setFrameForKinematics(Gleitlager->getFrameC());
  Gleitlager->setFrameOfReference(getFrameI());
  Gleitlager->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  Gleitlager->setRotation(new RotationAboutAxesXYZ<VecV>);
  Gleitlager->setMass(mScheibeGLS);
  Gleitlager->setInertiaTensor(ThetaScheibeGLS);

  Frustum* GleitlagerKontaktflaeche = new Frustum("GleitlagerKontaktflaeche");
  GleitlagerKontaktflaeche->setRadii(Vec(2, INIT, R_GLS));
  GleitlagerKontaktflaeche->setHeight(b_GLS);
  GleitlagerKontaktflaeche->setOutCont(false);
#ifdef HAVE_OPENMBVCPPINTERFACE
  GleitlagerKontaktflaeche->enableOpenMBV(_diffuseColor="[1;0;0.1]", _transparency=0.8);
#endif
  SqrMat AWK_GleitlagerKontaktflaeche(3, INIT, 0.);
  AWK_GleitlagerKontaktflaeche(2, 2) = 1.;
  AWK_GleitlagerKontaktflaeche(0, 0) = cos(M_PI / 2.);
  AWK_GleitlagerKontaktflaeche(1, 1) = cos(M_PI / 2.);
  AWK_GleitlagerKontaktflaeche(0, 1) = sin(M_PI / 2.);
  AWK_GleitlagerKontaktflaeche(1, 0) = -sin(M_PI / 2.);
  Gleitlager->addFrame(new FixedRelativeFrame("GleitlagerKontaktflaeche", Vec3(), AWK_GleitlagerKontaktflaeche));
  GleitlagerKontaktflaeche->setFrameOfReference(Gleitlager->getFrame("GleitlagerKontaktflaeche"));
  Gleitlager->addContour(GleitlagerKontaktflaeche);
  this->addObject(Gleitlager);

  /* Lager B */
  Vec3 VTemp;
  VTemp(0) = L;
  FixedRelativeFrame * lagerBRef = new FixedRelativeFrame("Lager_B_Frame", VTemp, SqrMat(3, EYE));
  addFrame(lagerBRef);
  RigidBody *LagerB = new RigidBody("LagerB");
  LagerB->setFrameOfReference(lagerBRef);
  addObject(LagerB);

  //Visualisierung von B
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Frustum> LagerBVis = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  LagerBVis->setBaseRadius(2.*radius);
  LagerBVis->setTopRadius(2.*radius);
  LagerBVis->setHeight(L/20.);
  LagerBVis->setScaleFactor(1.);
  LagerBVis->setDiffuseColor(1, 0, 0.1);
  LagerBVis->setTransparency(0.8);
  LagerBVis->setInitialTranslation(L/40.,0.,0.);
  LagerBVis->setInitialRotation(0.,M_PI/2.,0.);
  LagerB->setOpenMBVRigidBody(LagerBVis);
#endif

  Joint *blager = new Joint("Lager_B");
  blager->setForceDirection(Mat(3, 3, EYE));
  blager->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(StiffnessLagerB, DampingLagerB)));
  blager->connect(LagerB->getFrameC(), welle->getFrame("Ende"));
  blager->enableOpenMBVForce(1e-4);
  this->addLink(blager);

  /* Antrieb am Lager B */
  KineticExcitation *bantrieb = new KineticExcitation("Lager_B_Antrieb");
  bantrieb->setMomentFunction(new StepFunction<VecV(double)>(0.05, 0., AntriebsmomentLagerB));
  bantrieb->setMomentDirection("[0;1;0]");
  bantrieb->connect(welle->getFrame("Ende"));
  bantrieb->enableOpenMBVMoment(1e-4);
  this->addLink(bantrieb);

  KineticExcitation *bSchlag = new KineticExcitation("Lager_B_Schlag");
  bSchlag->setMomentFunction(new ConstantFunction<VecV(double)>(SchlagLagerB));
  bSchlag->setMomentDirection("[0;0;1]");
  bSchlag->connect(welle->getFrame("Ende"));
  bSchlag->enableOpenMBVForce(1e-4);
  this->addLink(bSchlag);

  /* Verbindung Schwungrad - Welle (EHD-Kontakt)*/
  // Contact Between journal and housing (EHD-Contact)
  EHDContact * ctBeamHou = new EHDContact("Contact_Beam_Housing");
  addLink(ctBeamHou);

  //create lubricant
  Lubricant lub;
  lub = Lubricant(0.0109, 778, 0, false, Lubricant::constVisc, Lubricant::constDen);

  // discretization
  EHDPressureElement ele("quad4", 4);
  ele.setLubricant(lub);

// Create computational mesh (half fluid domain)
  RowVec2 yb;
  yb(0) = 0;
  yb(1) = 2 * M_PI;
  RowVec2 zb;
  zb(0) = 0;
  zb(1) = b_GLS;
  MatVx2 xb(2);
  xb.set(0, yb);
  xb.set(1, zb);

  EHDMesh * msh = new EHDMesh(ele, xb, VecInt("[20; 5]"));

  msh->Boundary(EHDMesh::dbc, EHDMesh::x2m);
  msh->Boundary(EHDMesh::dbc, EHDMesh::x2p);    // z = -L / 2
  msh->Boundary(EHDMesh::per1, EHDMesh::x1m);   // y = 0
  msh->Boundary(EHDMesh::per2, EHDMesh::x1p);   // y = 2 * pi * R2

//  EHDForceLaw *fL = new EHDForceLaw();
  ctBeamHou->setMesh(msh);
  ctBeamHou->enableOpenMBVContactPoints(1e-5);
  ctBeamHou->enableOpenMBVNormalForce(5e-5);

  //contact kinematics (delivers the info for the mesh)
  ContactKinematicsCylinderFlexibleSolidCylinderHollowEHD *cK = new ContactKinematicsCylinderFlexibleSolidCylinderHollowEHD();
  ctBeamHou->connect(GleitlagerKontaktflaeche, welle->getContour("CylinderFlexible"), cK);

}


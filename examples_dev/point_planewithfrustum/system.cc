#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName, bool setValued) : DynamicSystemSolver(projectName) {

  setAccelerationOfGravity("[0; -9.81; 0]");

  bool considerRotation=true;

  double R=7e-2;
  double r=3.5e-2;
  double rho=.01e-2;
  double h=-6e-2;

  double rBar=.02;
  double lBar=.3;
  double rhoBar=786.;
  double mue=.1;

  double mBar=M_PI*rBar*rBar*lBar*rhoBar;
  cout << "mBar=" << mBar*1e3 << " [g]." << endl;
  SymMat inertiaBar(3, EYE);
  inertiaBar(0,0)=1./4.*mBar*rBar*rBar+1./12.*mBar*lBar*lBar;
  inertiaBar(1,1)=1./2.*mBar*rBar*rBar;
  inertiaBar(2,2)=1./4.*mBar*rBar*rBar+1./12.*mBar*lBar*lBar;
  cout << "inertiaBar=" << inertiaBar*1e3*1e6 << " [g*mm^2]." << endl;

  addFrame("I2", Vec("[-.2; -.25; .1]"), BasicRotAIKx(.3)*BasicRotAIKy(.7)*BasicRotAIKz(-0.7));
  addContour(new PlaneWithFrustum("Plane", R, r, h, rho), "[0; 0; 0]", BasicRotAIKz(M_PI/2.), getFrame("I2"));

#ifdef HAVE_OPENMBVCPPINTERFACE
  // nur fuer Visualisierung
  RigidBody * m = new RigidBody("PlaneContour");
  addObject(m);
  m->setFrameOfReference(getFrame("I2"));
  m->setFrameForKinematics(m->getFrame("C"));
  m->setMass(mBar);
  m->setInertiaTensor(inertiaBar);
  OpenMBV::Frustum * mVisu = new OpenMBV::Frustum();
  m->setOpenMBVRigidBody(mVisu);
  if (h<0) {
    mVisu->setBaseRadius(R+.01*r);
    mVisu->setInnerBaseRadius(R);
    mVisu->setTopRadius(r+.01*r);
    mVisu->setInnerTopRadius(r);
  }
  else {
    mVisu->setBaseRadius(R);
    mVisu->setTopRadius(r);
  }
  mVisu->setHeight(h);
  mVisu->setStaticColor(.3);
  mVisu->setInitialRotation(-M_PI/2., 0, 0);
  mVisu->setInitialTranslation(0, h, 0);
  m->getFrame("C")->enableOpenMBV(2.*rBar, .9);
#endif

  RigidBody * b = new RigidBody("Bar");
  b->setPlotFeature(plotRecursive, enabled);
  b->setPlotFeature(globalPosition, enabled);
  addObject(b);
  b->setFrameOfReference(getFrame("I"));
  b->setFrameForKinematics(b->getFrame("C"));
  Vec SysBar_r_cog_Top(3);
  SysBar_r_cog_Top(1)=lBar/2.;
  cout << "SysBar_r_cog_Top=" << SysBar_r_cog_Top << endl;
  b->addFrame("Top", SysBar_r_cog_Top, SqrMat(3, EYE));
  b->getFrame("Top")->setPlotFeature(globalPosition, enabled);
  b->setMass(mBar);
  b->setInertiaTensor(inertiaBar);
  b->setTranslation(new LinearTranslation("[1, 0, 0; 0, 1, 0; 0, 0, 1]"));
  if (considerRotation)
    b->setRotation(new CardanAngles());
  b->addContour(new Point("Point"), Vec(3), SqrMat(3), b->getFrame("Top"));
  if (considerRotation) {
    b->setInitialGeneralizedPosition("[.01; -.14; -.02; 0; 0; 0]");
    b->setInitialGeneralizedVelocity("[-1; 0; .5; 0; 2; 1]");
  }
  else {
    b->setInitialGeneralizedPosition("[[.01; -.14; -.02]");
    b->setInitialGeneralizedVelocity("[-1; 0; .5]");
  }
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Frustum * bVisu = new OpenMBV::Frustum();
  b->setOpenMBVRigidBody(bVisu);
  bVisu->setBaseRadius(rBar);
  bVisu->setTopRadius(rBar);
  bVisu->setHeight(lBar);
  bVisu->setInitialTranslation(0, -lBar/2., 0);
  bVisu->setInitialRotation(M_PI/2., 0, 0);

  b->getFrame("C")->enableOpenMBV(2.*rBar, .9);
  b->getFrame("Top")->enableOpenMBV(2.*rBar, .9);
#endif

  Contact * c = new Contact("ContactPointPlane");
  addLink(c);
  c->connect(b->getContour("Point"), getContour("Plane"));
  if (setValued) {
    c->setContactForceLaw(new UnilateralConstraint());
    c->setContactImpactLaw(new UnilateralNewtonImpact());
    c->setFrictionForceLaw(new SpatialCoulombFriction(mue));
  }
  else {
    c->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e5, 1e3));
    c->setFrictionForceLaw(new LinearRegularizedSpatialCoulombFriction(mue));
  }

}


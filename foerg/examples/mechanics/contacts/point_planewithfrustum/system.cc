#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/utils.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

#include <iostream>

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName, bool setValued) : DynamicSystemSolver(projectName) {

  MBSimEnvironment::getInstance()->setAccelerationOfGravity("[0; -9.81; 0]");

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

  addFrame(new FixedRelativeFrame("I2", Vec("[-.2; -.25; .1]"), BasicRotAIKx(.3)*BasicRotAIKy(.7)*BasicRotAIKz(-0.7)));
  addFrame(new FixedRelativeFrame("I3", "[0; 0; 0]", BasicRotAIKz(M_PI/2.), getFrame("I2")));
  addContour(new PlaneWithFrustum("Plane", R, r, h, rho, getFrame("I3")));

#ifdef HAVE_OPENMBVCPPINTERFACE
  // nur fuer Visualisierung
  RigidBody * m = new RigidBody("PlaneContour");
  addObject(m);
  m->setFrameOfReference(getFrame("I2"));
  m->setFrameForKinematics(m->getFrame("C"));
  m->setMass(mBar);
  m->setInertiaTensor(inertiaBar);
  boost::shared_ptr<OpenMBV::Frustum> mVisu = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
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
  mVisu->setDiffuseColor(0.3333,1,0.6666);
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
  b->addFrame(new FixedRelativeFrame("Top0", SysBar_r_cog_Top, SqrMat(3, EYE)));
  b->addFrame(new FixedRelativeFrame("Top1", SysBar_r_cog_Top+rBar*(+1.)*Vec("[1;0;0]"), SqrMat(3, EYE)));
  b->addFrame(new FixedRelativeFrame("Top2", SysBar_r_cog_Top+rBar*(-1.)*Vec("[1;0;0]"), SqrMat(3, EYE)));
  b->addFrame(new FixedRelativeFrame("Top3", SysBar_r_cog_Top+rBar*(+1.)*Vec("[0;0;1]"), SqrMat(3, EYE)));
  b->addFrame(new FixedRelativeFrame("Top4", SysBar_r_cog_Top+rBar*(-1.)*Vec("[0;0;1]"), SqrMat(3, EYE)));
  b->setMass(mBar);
  b->setInertiaTensor(inertiaBar);
  b->setTranslation(new TranslationAlongAxesXYZ<VecV>);
  if (considerRotation) {
    b->setRotation(new RotationAboutAxesXYZ<VecV>);
  }
  for (int i=0; i<5; i++)
    b->addContour(new Point("Point"+numtostr(i), b->getFrame("Top"+numtostr(i))));
  if (considerRotation) {
    b->setInitialGeneralizedPosition("[.01; -.14; -.02; 0; 0; 0]");
    b->setInitialGeneralizedVelocity("[-1; 0; .5; 0; 2; 1]");
  }
  else {
    b->setInitialGeneralizedPosition("[[.01; -.14; -.02]");
    b->setInitialGeneralizedVelocity("[-1; 0; .5]");
  }
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Frustum> bVisu = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  b->setOpenMBVRigidBody(bVisu);
  bVisu->setBaseRadius(rBar);
  bVisu->setTopRadius(rBar);
  bVisu->setHeight(lBar);
  bVisu->setInitialTranslation(0, -lBar/2., 0);
  bVisu->setInitialRotation(M_PI/2., 0, 0);

  b->getFrame("C")->enableOpenMBV(2.*rBar, .9);
  for (int i=0; i<5; i++)
    b->getFrame("Top"+numtostr(i))->enableOpenMBV(.2*rBar, .9);
#endif

  for (int i=1; i<5; i++) {
    Contact * c = new Contact("ContactPointPlane"+numtostr(i));
    addLink(c);
    c->connect(b->getContour("Point"+numtostr(i)), getContour("Plane"));
    if (setValued) {
      c->setNormalForceLaw(new UnilateralConstraint());
      c->setNormalImpactLaw(new UnilateralNewtonImpact());
      c->setTangentialForceLaw(new SpatialCoulombFriction(mue));
      c->setTangentialImpactLaw(new SpatialCoulombImpact(mue));
    }
    else {
      c->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5, 1e3)));
      c->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mue)));
    }
#ifdef HAVE_OPENMBVCPPINTERFACE
    c->enableOpenMBVContactPoints(.1*rBar);
#endif
  }

}


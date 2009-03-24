#include "system.h"
#include "mbsim/flexible_body/flexible_body_1s_21_rcm.h"
#include "mbsim/joint.h"
#include "mbsim/constitutive_laws.h"

#ifdef HAVE_AMVIS
using namespace AMVis;
#endif

System::System(const string &projectName) : MultiBodySystem(projectName) {

  this->setAccelerationOfGravity(Vec("[0; -10; 0]"));

  Vec q02D, u02D;

  double l0 = 1.5; // length
  double b0 = 0.1; // width
  double E = 5.e7; // E-Modul alu 7.e10; E-Modul iron 2.1e11; E-Modul rubber 5.e7
  double A = b0*b0; // cross-section area
  double I1 = 1./12.*b0*b0*b0*b0; // moment inertia
  double rho = 9.2e2; // density alu 2.7e3; density iron 7.9e3; density rubber 9.2e2
  int elements = 2; // number of finite elements


  /* 2D-Beam */ 
  FlexibleBody1s21RCM *rod2D = new FlexibleBody1s21RCM("Rod2D", true);
  rod2D->setLength(l0);
  rod2D->setEModul(E);
  rod2D->setCrossSectionalArea(A);
  rod2D->setMomentInertia(I1);
  rod2D->setDensity(rho);
  rod2D->setStationaryFrameOfReference(this->getFrame("I"));

  rod2D->setNumberElements(elements);
#ifdef HAVE_AMVIS
  rod2D->setAMVisCuboid(b0,b0);
#endif

  q02D = Vec(5*elements+3,INIT,0.);
  for(int i=1;i<=elements;i++) q02D(5*i) = l0*i/elements;

  u02D = Vec(5*elements+3,INIT,0.);
  for(int i=0;i<=elements;i++) {
    u02D(5*i+2) = 0.1;
    u02D(5*i+1) = 0.1*l0*i/elements;
  }

  ContourPointData cpdata;
  cpdata.alpha = Vec(1,INIT,0.);
  cpdata.type = CONTINUUM;
  rod2D->addFrame("RJ",cpdata);
  Joint *joint2D = new Joint("Clamping_2D");
  joint2D->connect(this->getFrame("I"),rod2D->getFrame("RJ")); 
  joint2D->setForceDirection(Mat("[1,0; 0,1; 0,0]"));
  joint2D->setForceLaw(new BilateralConstraint);
  joint2D->setImpactForceLaw(new BilateralImpact);
  joint2D->setMomentDirection("[0; 0; 1]");
  joint2D->setMomentLaw(new BilateralConstraint);
  joint2D->setImpactMomentLaw(new BilateralImpact);
  this->addLink(joint2D);
 
  rod2D->setq0(q02D);
  rod2D->setu0(u02D);
#ifdef HAVE_AMVIS
  rod2D->createAMVisBody(true);
#endif
  this->addObject(rod2D);
}


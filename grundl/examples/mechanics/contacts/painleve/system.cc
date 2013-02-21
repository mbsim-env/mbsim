#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/multi_contact.h"
#include "mbsim/constitutive_laws.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#endif

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Gravitation
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(Vec("[0;-10;0]"));

  // Koerper
  double alpha0 =  M_PI/2.0*0.6;

  RigidBody *stab = new RigidBody("Stab");
  addObject(stab);
  
  double lStab = 0.5;
  double hStab = 0.02;
  double mStab = 1;
  
  Vec WrOSStab(3);
  WrOSStab(0) = lStab;
  WrOSStab(1) = 0.5*sqrt(lStab*lStab+hStab*hStab)*sin(alpha0)+0.015;
  addFrame("D",WrOSStab,SqrMat(3,EYE));
  
  stab->setFrameOfReference(this->getFrame("D"));
  stab->setFrameForKinematics(stab->getFrame("C"));
  
  stab->setMass(mStab);
  SymMat Theta(3);
  double JStab = 1./12. * mStab * lStab * lStab; 
  Theta(2,2) = JStab;
  stab->setInertiaTensor(Theta);
  stab->setTranslation(new LinearTranslation("[1,0; 0,1; 0,0]"));
  stab->setRotation(new RotationAboutFixedAxis("[0; 0; 1]"));
  
  Vec q0Stab(3);
  q0Stab(2) = alpha0;
  stab->setInitialGeneralizedPosition(q0Stab);
  
  Vec v0Stab(3);
  v0Stab(0) = -4.;
  stab->setInitialGeneralizedVelocity(v0Stab);
  
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid* cuboid = new OpenMBV::Cuboid;
  cuboid->setLength(lStab,hStab,0.1);
  cuboid->setStaticColor(0.1);
  stab->setOpenMBVRigidBody(cuboid);
#endif

  // Contouren 
  Vec rSP(3);
  rSP(0) = -lStab/2.;
  rSP(1) = -hStab/2.;
  Point *point = new Point("PunktUntenLinks");
  stab->addContour(point,rSP,SqrMat(3,EYE));
  
  rSP(0) = -lStab/2.;
  rSP(1) =  hStab/2.;
  point = new Point("PunktUntenRechts");
  stab->addContour(point,rSP,SqrMat(3,EYE));
  
  rSP(0) =  lStab/2.;
  rSP(1) = -hStab/2.;
  point = new Point("PunktObenLinks");
  stab->addContour(point,rSP,SqrMat(3,EYE));
  
  rSP(0) = lStab/2.;
  rSP(1) = hStab/2.;
  point = new Point("PunktObenRechts");
  stab->addContour(point,rSP,SqrMat(3,EYE));

  Plane *line = new Plane("Grund");
  SqrMat lineRot("[0,1,0;1,0,0;0,0,1]");
  addContour(line,Vec(3,INIT,0),lineRot);

  // Contacts
  Contact *cnf = new Contact("KontaktUntenLinks");
  cnf->connect(stab->getContour("PunktUntenLinks"), getContour("Grund"));
  cnf->setContactForceLaw(new UnilateralConstraint());
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setFrictionForceLaw(new PlanarCoulombFriction(1.5));
  cnf->setFrictionImpactLaw(new PlanarCoulombImpact(1.5));
  addLink(cnf);
  
  cnf = new Contact("KontaktObenLinks");
  cnf->connect(stab->getContour("PunktObenLinks"), getContour("Grund"));
  cnf->setContactForceLaw(new UnilateralConstraint());
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setFrictionForceLaw(new PlanarCoulombFriction(0.2));
  cnf->setFrictionImpactLaw(new PlanarCoulombImpact(0.2));
  addLink(cnf);
  
  cnf = new Contact("KontaktObenRechts");
  cnf->connect(stab->getContour("PunktObenRechts"), getContour("Grund"));
  cnf->setContactForceLaw(new UnilateralConstraint());
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setFrictionForceLaw(new PlanarCoulombFriction(0.2));
  cnf->setFrictionImpactLaw(new PlanarCoulombImpact(0.2));
  addLink(cnf);
  
  cnf = new Contact("KontaktUntenRechts");
  cnf->connect(stab->getContour("PunktUntenRechts"), getContour("Grund"));
  cnf->setContactForceLaw(new UnilateralConstraint());
  cnf->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setFrictionForceLaw(new PlanarCoulombFriction(1.5));
  cnf->setFrictionImpactLaw(new PlanarCoulombImpact(1.5));
  addLink(cnf);
}


#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/links/contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/functions/kinematics/kinematics.h"

#include "openmbvcppinterface/cuboid.h"

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
  addFrame(new FixedRelativeFrame("D",WrOSStab,SqrMat(3,EYE)));
  
  stab->setFrameOfReference(this->getFrame("D"));
  stab->setFrameForKinematics(stab->getFrame("C"));
  
  stab->setMass(mStab);
  SymMat Theta(3);
  double JStab = 1./12. * mStab * lStab * lStab; 
  Theta(2,2) = JStab;
  stab->setInertiaTensor(Theta);
  stab->setTranslation(new TranslationAlongAxesXY<VecV>);
  stab->setRotation(new RotationAboutZAxis<VecV>);
  
  Vec q0Stab(3);
  q0Stab(2) = alpha0;
  stab->setGeneralizedInitialPosition(q0Stab);
  
  Vec v0Stab(3);
  v0Stab(0) = -4.;
  stab->setGeneralizedInitialVelocity(v0Stab);
  
  std::shared_ptr<OpenMBV::Cuboid> cuboid = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  cuboid->setLength(lStab,hStab,0.1);
  cuboid->setDiffuseColor(0.3333,1,0.3333);
  stab->setOpenMBVRigidBody(cuboid);

  // Contouren 
  Vec rSP(3);
  rSP(0) = -lStab/2.;
  rSP(1) = -hStab/2.;
  stab->addFrame(new FixedRelativeFrame("PUL",rSP,SqrMat(3,EYE)));
  stab->addContour(new Point("PunktUntenLinks",stab->getFrame("PUL")));
  
  rSP(0) = -lStab/2.;
  rSP(1) =  hStab/2.;
  stab->addFrame(new FixedRelativeFrame("PUR",rSP,SqrMat(3,EYE)));
  stab->addContour(new Point("PunktUntenRechts",stab->getFrame("PUR")));
  
  rSP(0) =  lStab/2.;
  rSP(1) = -hStab/2.;
  stab->addFrame(new FixedRelativeFrame("POL",rSP,SqrMat(3,EYE)));
  stab->addContour(new Point("PunktObenLinks",stab->getFrame("POL")));
  
  rSP(0) = lStab/2.;
  rSP(1) = hStab/2.;
  stab->addFrame(new FixedRelativeFrame("POR",rSP,SqrMat(3,EYE)));
  stab->addContour(new Point("PunktObenRechts",stab->getFrame("POR")));

  addFrame(new FixedRelativeFrame("P",Vec(3,INIT,0),SqrMat("[0,1,0;1,0,0;0,0,1]")));
  addContour(new Plane("Grund",getFrame("P")));

  // Contacts
  Contact *cnf = new Contact("KontaktUntenLinks");
  cnf->connect(stab->getContour("PunktUntenLinks"), getContour("Grund"));
  cnf->setNormalForceLaw(new UnilateralConstraint());
  cnf->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setTangentialForceLaw(new PlanarCoulombFriction(1.5));
  cnf->setTangentialImpactLaw(new PlanarCoulombImpact(1.5));
  addLink(cnf);
  
  cnf = new Contact("KontaktObenLinks");
  cnf->connect(stab->getContour("PunktObenLinks"), getContour("Grund"));
  cnf->setNormalForceLaw(new UnilateralConstraint());
  cnf->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setTangentialForceLaw(new PlanarCoulombFriction(0.2));
  cnf->setTangentialImpactLaw(new PlanarCoulombImpact(0.2));
  addLink(cnf);
  
  cnf = new Contact("KontaktObenRechts");
  cnf->connect(stab->getContour("PunktObenRechts"), getContour("Grund"));
  cnf->setNormalForceLaw(new UnilateralConstraint());
  cnf->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setTangentialForceLaw(new PlanarCoulombFriction(0.2));
  cnf->setTangentialImpactLaw(new PlanarCoulombImpact(0.2));
  addLink(cnf);
  
  cnf = new Contact("KontaktUntenRechts");
  cnf->connect(stab->getContour("PunktUntenRechts"), getContour("Grund"));
  cnf->setNormalForceLaw(new UnilateralConstraint());
  cnf->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
  cnf->setTangentialForceLaw(new PlanarCoulombFriction(1.5));
  cnf->setTangentialImpactLaw(new PlanarCoulombImpact(1.5));
  addLink(cnf);

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
}

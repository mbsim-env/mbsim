#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/constraints/generalized_gear_constraint.h"
#include "mbsimPowertrain/differential_gear.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/constant_function.h"

#include "openmbvcppinterface/frustum.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimPowertrain;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  double R1 = 0.02;
  double m1 = 1;
  double J = 0.5*m1*R1*R1; 
  SymMat Theta(3);
  double l = 0.1;

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  Vec r(3);
  r(0) = l/2;
  addFrame(new FixedRelativeFrame("Q",r,BasicRotAKIy(M_PI/2)));

  RigidBody* shaft1 = new RigidBody("Shaft1");
  addObject(shaft1);


  shaft1->setFrameOfReference(getFrame("Q"));
  shaft1->setFrameForKinematics(shaft1->getFrame("C"));


  shaft1->setMass(m1);
  Theta(2,2) = J;
  shaft1->setInertiaTensor(Theta);
  shaft1->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  r.init(0);
  r(2) = l/2;
  shaft1->addFrame(new FixedRelativeFrame("Q",r,SqrMat(3,EYE)));
  shaft1->getFrame("Q")->enableOpenMBV(0.3);
  shaft1->getFrame("C")->enableOpenMBV(0.3);

  DifferentialGear* differentialGear = new DifferentialGear("DifferentialGear");
  addGroup(differentialGear);
  double R2 = differentialGear->getRadiusInputShaft();

  GeneralizedGearConstraint *constraint = new GeneralizedGearConstraint("C1");
  addConstraint(constraint);
  constraint->setDependentRigidBody(shaft1);
  constraint->addIndependentRigidBody(static_cast<RigidBody*>(differentialGear->getObject("InputShaft")),-R2/R1);

  KineticExcitation* ke;
  ke = new KineticExcitation("MAn");
  addLink(ke);
  ke->connect(shaft1->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(1.1/100.));

  ke = new KineticExcitation("MAbL");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(differentialGear->getObject("LeftOutputShaft"))->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(0.99/100.));

  ke = new KineticExcitation("MAbR");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(differentialGear->getObject("RightOutputShaft"))->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(1/100.));

  std::shared_ptr<OpenMBV::Frustum> cylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  cylinder->setTopRadius(R1);
  cylinder->setBaseRadius(R1);
  cylinder->setHeight(l);
  cylinder->setDiffuseColor(0.1,1,1);
  shaft1->setOpenMBVRigidBody(cylinder);
  cylinder->setInitialTranslation(0,0,l/2);

  setPlotFeatureRecursive(generalizedPosition,enabled);
  setPlotFeatureRecursive(generalizedVelocity,enabled);
  setPlotFeatureRecursive(generalizedRelativePosition,enabled);
  setPlotFeatureRecursive(generalizedRelativeVelocity,enabled);
  setPlotFeatureRecursive(generalizedForce,enabled);
}


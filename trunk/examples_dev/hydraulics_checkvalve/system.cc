#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/hydnode_mec.h"
#include "mbsim/userfunction.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsimHydraulics/checkvalve.h"
#include "mbsim/rigid_body.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/utils/function_library.h"
#include "mbsimControl/object_sensors.h"

#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool bilateral, bool unilateral) : Group(name) {

  getFrame("I")->enableOpenMBV(.01, 1);

  addFrame("ref", Vec("[.01; .02; .00]"), BasicRotAIKy(1)*BasicRotAIKz(-1.));
  getFrame("ref")->enableOpenMBV(.005, 1);

  RigidBody * b = new RigidBody("Body");
  addObject(b);
  b->setMass(.1);
  b->setInertiaTensor(SymMat(3, EYE));
  b->setFrameOfReference(getFrame("ref"));
  b->setFrameForKinematics(b->getFrame("C"));
  b->addFrame("ref", Vec("[.01; .02; .00]"), BasicRotAIKy(-2.)*BasicRotAIKz(-1.));

  b->getFrame("ref")->enableOpenMBV(.01, 1);

  Checkvalve * lCV = new Checkvalve("lCV");
  addGroup(lCV);
  lCV->getLine()->setDiameter(4e-3);
  lCV->getLine()->setLength(.05);
  if (unilateral)
    lCV->setVariablePressureLossCheckvalve(new VariablePressureLossCheckvalveGamma("CheckvalveGamma", lCV->getXOpen(), 1e-4, 6e-3, .1, M_PI/4.));
  else
    lCV->setVariablePressureLossCheckvalve(new RegularizedVariablePressureLossCheckvalveGamma("CheckvalveGamma", lCV->getXOpen(), 1e-4, 6e-3, .1, M_PI/4.));

  lCV->setMaximalOpening(.003);
  lCV->setFrameOfReference(b->getFrame("ref"));
  if (unilateral) {
    lCV->getSeatContact()->setContactForceLaw(new UnilateralConstraint);
    lCV->getSeatContact()->setContactImpactLaw(new UnilateralNewtonImpact);
    lCV->getMaximalContact()->setContactForceLaw(new UnilateralConstraint);
    lCV->getMaximalContact()->setContactImpactLaw(new UnilateralNewtonImpact);
  }
  else {
    double c=1e5;
    double d=1e3;
    lCV->getSeatContact()->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(c, d)));
    lCV->getMaximalContact()->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(c, d)));
  }
  lCV->getBall()->setMass(0.1);
  lCV->getBall()->setInertiaTensor(SymMat(3, EYE)*1e-5);
  lCV->getBall()->setPlotFeature(rightHandSide, enabled);
  lCV->getLine()->setPlotFeature(rightHandSide, enabled);

  ConstrainedNodeMec * n1 = new ConstrainedNodeMec("n1");
  n1->setpFunction(new Function1_SS_from_VS(new  TabularFunction1_VS(Vec("[0; .9; 1.1; 2.9; 3.1; 5]")*.1, "[4e5; 4e5; 2e5; 2e5; 4e5; 4e5]")));
  addLink(n1);
  n1->addOutFlow(lCV->getLine());

  ConstrainedNodeMec * n2 = new ConstrainedNodeMec("n2");
  addLink(n2);
  n2->setpFunction(new ConstantFunction1<double,double>(3e5));
  n2->addInFlow(lCV->getLine());

  lCV->getSeatContact()->enableOpenMBVContactPoints(.0005);
  lCV->getMaximalContact()->enableOpenMBVContactPoints(.0005);

}

#include "system.h"

#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/controlvalve43.h"
#include "mbsim/utils/function_library.h"
#include "mbsimControl/function_sensor.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool bilateral, bool unilateral) : Group(name) {

  RigidLine * lp = new RigidLine("lP");
  addObject(lp);
  lp->setFrameOfReference(getFrame("I"));
  lp->setDirection("[0; 0; 0]");
  lp->setDiameter(5e-3);
  lp->setLength(.1);

  RigidLine * la = new RigidLine("lA");
  addObject(la);
  la->setFrameOfReference(getFrame("I"));
  la->setDirection("[0; 0; 0]");
  la->setDiameter(5e-3);
  la->setLength(.1);

  RigidLine * lb = new RigidLine("lB");
  addObject(lb);
  lb->setFrameOfReference(getFrame("I"));
  lb->setDirection("[0; 0; 0]");
  lb->setDiameter(5e-3);
  lb->setLength(.1);

  RigidLine * lt = new RigidLine("lT");
  addObject(lt);
  lt->setFrameOfReference(getFrame("I"));
  lt->setDirection("[0; 0; 0]");
  lt->setDiameter(5e-3);
  lt->setLength(.1);
  
  Controlvalve43 * cv = new Controlvalve43("Valve43");
  addGroup(cv);
  if (unilateral)
    cv->setSetValued();
  cv->setLength(.05);
  cv->setDiameter(2e-3);
  cv->setPInflow(lp);
  cv->setAOutflow(la);
  cv->setBOutflow(lb);
  cv->setTOutflow(lt);
  cv->setAlpha(.9);
  cv->setMinimalRelativeAlpha(.05);
  cv->setOffset(.05);
  cv->setPARelativeAlphaFunction(new Function1_SS_from_VS(new TabularFunction1_VS(Vec("[0; .2; .45; 1]"), "[1; 1; 0; 0]")));
  FunctionSensor * cvs = new FunctionSensor("Valve43Position");
  addLink(cvs);
  cvs->setFunction(new TabularFunction1_VS(Vec("[0; .3; .7; 1]"), "[0; 0; 1; 1]"));
  cv->setRelativePositionSignal(cvs);

  ConstrainedNode * nP = new ConstrainedNode("n_source_P");
  nP->setpFunction(new ConstantFunction1<double, double>(5e5));
  addLink(nP);
  nP->addOutFlow(lp);

  ConstrainedNode * nA = new ConstrainedNode("n_source_A");
  nA->setpFunction(new ConstantFunction1<double, double>(3e5));
  addLink(nA);
  nA->addInFlow(la);

  ConstrainedNode * nB = new ConstrainedNode("n_source_B");
  nB->setpFunction(new ConstantFunction1<double, double>(2e5));
  addLink(nB);
  nB->addInFlow(lb);

  ConstrainedNode * nT = new ConstrainedNode("n_source_T");
  nT->setpFunction(new ConstantFunction1<double, double>(1e5));
  addLink(nT);
  nT->addInFlow(lt);

}

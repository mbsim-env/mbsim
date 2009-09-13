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
  
  Controlvalve43 * cv = new Controlvalve43("Valve43");
  if (unilateral)
    cv->setSetValued();
  addGroup(cv);
  cv->setLength(.05);
  cv->setDiameter(2e-3);
  cv->setFrameOfReference(getFrame("I"));
  cv->setLinePDirection(Vec(3, INIT, 0));
  cv->setLineADirection(Vec(3, INIT, 0));
  cv->setLineBDirection(Vec(3, INIT, 0));
  cv->setLineTDirection(Vec(3, INIT, 0));
  cv->setLinePDiameter(5e-3);
  cv->setLineADiameter(5e-3);
  cv->setLineBDiameter(5e-3);
  cv->setLineTDiameter(5e-3);
  cv->setLinePLength(.1);
  cv->setLineALength(.1);
  cv->setLineBLength(.1);
  cv->setLineTLength(.1);
  cv->setAlpha(.9);
  cv->setMinimalRelativeArea(.05);
  cv->setOffset(.05);
  cv->setPARelativeAreaFunction(new Function1_SS_from_VS(new TabularFunction1_VS(Vec("[0; .2; .45; 1]"), "[1; 1; 0; 0]")));
  FunctionSensor * cvs = new FunctionSensor("Valve43Position");
  addLink(cvs);
  cvs->setFunction(new TabularFunction1_VS(Vec("[0; .3; .7; 1]"), "[0; 0; 1; 1]"));
  cv->setRelativePositionSignal(cvs);

  ConstrainedNode * nP = new ConstrainedNode("nP");
  nP->setpFunction(new ConstantFunction1<double, double>(5e5));
  addLink(nP);
  nP->addOutFlow(cv->getLineP());

  ConstrainedNode * nA = new ConstrainedNode("nA");
  nA->setpFunction(new ConstantFunction1<double, double>(3e5));
  addLink(nA);
  nA->addInFlow(cv->getLineA());

  ConstrainedNode * nB = new ConstrainedNode("nB");
  nB->setpFunction(new ConstantFunction1<double, double>(2e5));
  addLink(nB);
  nB->addInFlow(cv->getLineB());

  ConstrainedNode * nT = new ConstrainedNode("nT");
  nT->setpFunction(new ConstantFunction1<double, double>(1e5));
  addLink(nT);
  nT->addInFlow(cv->getLineT());

}

#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydline_galerkin.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/utils/function_library.h"
#include "mbsimControl/function_sensor.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool bilateral, bool unilateral) : Group(name) {
  
  HydLine * l12 = new HydLine("l12");
  addObject(l12);
  l12->setDiameter(4e-3);
  l12->setLength(.4);
  l12->addPressureLoss(new PressureLossZeta("zeta1", 15));

  HydLine * l23 = new HydLine("l23");
  addObject(l23);
  l23->setDiameter(3e-3);
  l23->setLength(.1);
  FunctionSensor * l23s = new FunctionSensor("Valveposition23");
  addLink(l23s);
  l23s->setFunction(new TabularFunction1_VS(Vec("[0; .19; .21; .29; .31; .69; .71; .79; .81; 1]"), "[0;   0;   1;   1;   0;  0;    1;   1; 0; 0]"));
  if (unilateral)
    l23->addPressureLoss(new VariablePressureLossAreaZeta("zeta1", 7, .01, l23s));
  else
    l23->addPressureLoss(new RegularizedVariablePressureLossAreaZeta("zeta1", 7, .01, l23s));

  HydLine * l34 = new HydLine("l34");
  addObject(l34);
  l34->setDiameter(5e-3);
  l34->setLength(.5);
  l34->addPressureLoss(new PressureLossZeta("zeta1", 15));

  HydNodeConstrained * n1 = new HydNodeConstrained("n1");
  n1->setpFunction(new Function1_VS_to_SS(new TabularFunction1_VS(Vec("[0; .35; .65; 1]"), "[5e5; 5e5; 1e5; 1e5]")));
  addLink(n1);
  n1->addOutFlow(l12);

  HydNode * n2;
  if (bilateral)
    n2 = new HydNodeRigid("n2");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n2");
    nTmp->setVolume(3e-6);
    nTmp->setFracAir(.02);
    nTmp->setp0(5e5);
    n2 = nTmp;
  }
  addLink(n2);
  n2->addInFlow(l12);
  n2->addOutFlow(l23);

  HydNode * n3;
  if (bilateral)
    n3 = new HydNodeRigid("n3");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n3");
    nTmp->setVolume(2e-6);
    nTmp->setFracAir(.02);
    nTmp->setp0(2.5e5);
    n3 = nTmp;
  }
  addLink(n3);
  n3->addInFlow(l23);
  n3->addOutFlow(l34);

  HydNodeConstrained * n4 = new HydNodeConstrained("n4");
  n4->setpFunction(new ConstantFunction1<double, double>(2.5e5));
  addLink(n4);
  n4->addInFlow(l34);

}

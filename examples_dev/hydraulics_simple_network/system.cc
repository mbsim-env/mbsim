#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsim/userfunction.h"
#include "mbsimHydraulics/pressure_loss.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool unilateral) : Group(name) {

  HydLine * l12a = new HydLine("l12a");
  addObject(l12a);
  l12a->setDiameter(5e-3);
  l12a->setLength(.7);
  l12a->addPressureLoss(new PressureLossZeta("zeta1", 14));

  HydLine * l12b = new HydLine("l12b");
  addObject(l12b);
  l12b->setDiameter(3e-3);
  l12b->setLength(.7);
  l12b->addPressureLoss(new PressureLossZeta("zeta1", 5));

  HydLine * l15 = new HydLine("l15");
  addObject(l15);
  l15->setDiameter(4e-3);
  l15->setLength(.2);
  l15->addPressureLoss(new PressureLossZeta("zeta1", 3));

  HydLine * l16 = new HydLine("l16");
  addObject(l16);
  l16->setDiameter(4e-3);
  l16->setLength(.2);
  l16->addPressureLoss(new PressureLossZeta("zeta1", 3));

  HydLine * l23 = new HydLine("l23");
  addObject(l23);
  l23->setDiameter(7e-3);
  l23->setLength(.7);
  l23->addPressureLoss(new PressureLossZeta("zeta1", 1.5));

  HydLine * l34 = new HydLine("l34");
  addObject(l34);
  l34->setDiameter(4e-3);
  l34->setLength(.5);
  l34->addPressureLoss(new PressureLossZeta("zeta1", 5));

  HydLine * l35 = new HydLine("l35");
  addObject(l35);
  l35->setDiameter(3e-3);
  l35->setLength(.25);
  l35->addPressureLoss(new PressureLossZeta("zeta1", 7));

  HydLine * l45 = new HydLine("l45");
  addObject(l45);
  l45->setDiameter(7e-3);
  l45->setLength(.7);
  l45->addPressureLoss(new PressureLossZeta("zeta1", 19));

  HydLine * l56 = new HydLine("l56");
  addObject(l56);
  l56->setDiameter(2e-3);
  l56->setLength(1.7);
  l56->addPressureLoss(new PressureLossZeta("zeta1", 10));

  HydNodeEnvironment * n1 = new HydNodeEnvironment("n1");
  addLink(n1);
  n1->enableOpenMBV(.025, 0, 10e5, "[-.1; 0; 0]");
  n1->addOutFlow(l12a);
  n1->addOutFlow(l12b);
  n1->addOutFlow(l15);
  n1->addOutFlow(l16);

  HydNodeConstrained * n2 = new HydNodeConstrained("n2");
  addLink(n2);
  n2->enableOpenMBV(.025, 0, 10e5, "[-.05; .1; 0]");
  n2->setpFunction(new ConstantFunction1<double, double>(5e5));
  n2->addInFlow(l12a);
  n2->addInFlow(l12b);
  n2->addOutFlow(l23);

  HydNode * n3;
  if (unilateral)
    n3 = new HydNodeRigid("n3");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n3");
    nTmp->setVolume(3e-6);
    nTmp->setFracAir(0.02);
    nTmp->setp0(1e5);
    n3 = nTmp;
  }
  addLink(n3);
  n3->enableOpenMBV(.025, 0, 10e5, "[.05; .1; 0]");
  n3->addInFlow(l23);
  n3->addOutFlow(l34);
  n3->addOutFlow(l35);

  HydNode * n4;
  if (unilateral)
    n4 = new HydNodeRigid("n4");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n4");
    nTmp->setVolume(3e-5);
    nTmp->setFracAir(0.01);
    nTmp->setp0(7e5);
    n4 = nTmp;
  }
  addLink(n4);
  n4->enableOpenMBV(.025, 0, 10e5, "[.1; 0; 0]");
  n4->addInFlow(l34);
  n4->addOutFlow(l45);

  HydNode * n5;
  if (unilateral)
    n5 = new HydNodeRigid("n5");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n5");
    nTmp->setVolume(4e-5);
    nTmp->setFracAir(0.02);
    nTmp->setp0(2e5);
    n5 = nTmp;
  }
  addLink(n5);
  n5->enableOpenMBV(.025, 0, 10e5, "[.05; -.1; 0]");
  n5->addInFlow(l15);
  n5->addInFlow(l35);
  n5->addInFlow(l45);
  n5->addOutFlow(l56);

  HydNode * n6;
  if (unilateral)
    n6 = new HydNodeRigid("n6");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n6");
    nTmp->setVolume(3e-5);
    nTmp->setFracAir(0.02);
    nTmp->setp0(3e5);
    n6 = nTmp;
  }
  addLink(n6);
  n6->enableOpenMBV(.025, 0, 10e5, "[-.05; -.1; 0]");
  n6->addInFlow(l16);
  n6->addInFlow(l56);
}

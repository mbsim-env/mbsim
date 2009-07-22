#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydline_galerkin.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsim/userfunction.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool setvalued, bool unilateral) : Group(name) {

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

  HydLine * l35;
  if (unilateral)
    l35 = new HydLineValveBilateral("l35");
  else
    l35 = new HydLineValve("l35");
  addObject(l35);
  l35->setDiameter(3e-3);
  l35->setLength(.25);
  FuncTable * area35 = new FuncTable();
  area35->setXY(Vec("[0; .09; .11; .39; .41; .69; .71; 0.9; 1]"), Vec("[0;   0;   1;   1;   0;  0;    1;   1;  0]"));
  l35->addPressureLoss(new PressureLossZetaVarArea("zeta1", 7, area35, .001));

  HydLineGalerkin * l45 = new HydLineGalerkin("l45");
  addObject(l45);
  l45->setDiameter(7e-3);
  l45->setLength(1.4);
  l45->setp0(2e5);
  l45->setFracAir(0.02);
  l45->setdh(0);
  l45->setDLehr(0.03);
  l45->setAnsatzFunction(HydLineGalerkin::Harmonic, 6);
  l45->setRelativePlotPoints(Vec("[0; .25; .5; .75; 1]"));

  HydLine * l56;
  if (unilateral)
    l56 = new HydLineValveBilateral("l56");
  else
    l56 = new HydLineValve("l56");
  addObject(l56);
  l56->setDiameter(2e-3);
  l56->setLength(1.7);
  FuncTable * area56 = new FuncTable();
  area56->setXY(Vec("[0; .2; .4; .89; .91; 1.29; 1.3; 1.8; 2]"), Vec("[0;   0;   1;   1;   0;  0;    1;   1;  0]"));
  l56->addPressureLoss(new PressureLossZetaVarArea("zeta1", 10, area56, .001));

  HydNodeConstrained * n1 = new HydNodeConstrained("n1");
  n1->setpFunction(new FuncHarmonic(Vec("3e5"), 8.*M_PI, 0, Vec("5e5")));
  addLink(n1);
  n1->addOutFlow(l12a);
  n1->addOutFlow(l12b);
  n1->addOutFlow(l15);
  n1->addOutFlow(l16);

  HydNodeConstrained * n2 = new HydNodeConstrained("n2");
  addLink(n2);
  n2->setpFunction(new FuncConst(Vec("5e5")));
  n2->addInFlow(l12a);
  n2->addInFlow(l12b);
  n2->addOutFlow(l23);

  HydNode * n3;
  if (setvalued)
    n3 = new HydNodeRigid("n3");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n3");
    nTmp->setVolume(3e-6);
    nTmp->setFracAir(.02);
    nTmp->setp0(1e5);
    n3 = nTmp;
  }
  addLink(n3);
  n3->addInFlow(l23);
  n3->addOutFlow(l34);
  n3->addOutFlow(l35);

  HydNode * n4;
  if (setvalued)
    n4 = new HydNodeRigid("n4");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n4");
    nTmp->setVolume(3e-5);
    nTmp->setFracAir(.01);
    nTmp->setp0(7e5);
    n4 = nTmp;
  }
  addLink(n4);
  n4->addInFlow(l34);
  n4->addOutFlow(l45);

  HydNode * n5;
  if (setvalued)
    n5 = new HydNodeRigid("n5");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n5");
    nTmp->setVolume(4e-5);
    nTmp->setFracAir(.02);
    nTmp->setp0(2e5);
    n5 = nTmp;
  }
  addLink(n5);
  n5->addInFlow(l15);
  n5->addInFlow(l35);
  n5->addInFlow(l45);
  n5->addOutFlow(l56);

  HydNode * n6;
  if (setvalued)
    n6 = new HydNodeRigid("n6");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n6");
    nTmp->setVolume(3e-5);
    nTmp->setFracAir(.02);
    nTmp->setp0(3e5);
    n6 = nTmp;
  }
  addLink(n6);
  n6->addInFlow(l16);
  n6->addInFlow(l56);
}

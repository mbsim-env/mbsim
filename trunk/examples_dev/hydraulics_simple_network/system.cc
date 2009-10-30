#include "system.h"

#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/pressure_loss.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimHydraulics;

System::System(const string &name, bool unilateral) : Group(name) {

  RigidLine * l12a = new RigidLine("l12a");
  addObject(l12a);
  l12a->setFrameOfReference(getFrame("I"));
  l12a->setDirection("[1; 0; 0]");
  l12a->setDiameter(5e-3);
  l12a->setLength(.7);
  ZetaLinePressureLoss * pl_l12a = new ZetaLinePressureLoss();
  pl_l12a->setZeta(14);
  l12a->setLinePressureLoss(pl_l12a);

  RigidLine * l12b = new RigidLine("l12b");
  addObject(l12b);
  l12b->setFrameOfReference(getFrame("I"));
  l12b->setDirection("[1; 0; 0]");
  l12b->setDiameter(3e-3);
  l12b->setLength(.7);
  ZetaLinePressureLoss * pl_l12b = new ZetaLinePressureLoss();
  pl_l12b->setZeta(5);
  l12b->setLinePressureLoss(pl_l12b);

  RigidLine * l15 = new RigidLine("l15");
  addObject(l15);
  l15->setFrameOfReference(getFrame("I"));
  l15->setDirection("[1; 0; 0]");
  l15->setDiameter(4e-3);
  l15->setLength(.2);
  ZetaLinePressureLoss * pl_l15 = new ZetaLinePressureLoss();
  pl_l15->setZeta(3);
  l15->setLinePressureLoss(pl_l15);

  RigidLine * l16 = new RigidLine("l16");
  addObject(l16);
  l16->setFrameOfReference(getFrame("I"));
  l16->setDirection("[1; 0; 0]");
  l16->setDiameter(4e-3);
  l16->setLength(.2);
  ZetaLinePressureLoss * pl_l16 = new ZetaLinePressureLoss();
  pl_l16->setZeta(3);
  l16->setLinePressureLoss(pl_l16);

  RigidLine * l23 = new RigidLine("l23");
  addObject(l23);
  l23->setFrameOfReference(getFrame("I"));
  l23->setDirection("[1; 0; 0]");
  l23->setDiameter(7e-3);
  l23->setLength(.7);
  ZetaLinePressureLoss * pl_l23 = new ZetaLinePressureLoss();
  pl_l23->setZeta(1.5);
  l23->setLinePressureLoss(pl_l23);

  RigidLine * l34 = new RigidLine("l34");
  addObject(l34);
  l34->setFrameOfReference(getFrame("I"));
  l34->setDirection("[1; 0; 0]");
  l34->setDiameter(4e-3);
  l34->setLength(.5);
  ZetaLinePressureLoss * pl_l34 = new ZetaLinePressureLoss();
  pl_l34->setZeta(5);
  l34->setLinePressureLoss(pl_l34);

  RigidLine * l35 = new RigidLine("l35");
  addObject(l35);
  l35->setFrameOfReference(getFrame("I"));
  l35->setDirection("[1; 0; 0]");
  l35->setDiameter(3e-3);
  l35->setLength(.25);
  ZetaLinePressureLoss * pl_l35 = new ZetaLinePressureLoss();
  pl_l35->setZeta(7);
  l35->setLinePressureLoss(pl_l35);

  RigidLine * l45 = new RigidLine("l45");
  addObject(l45);
  l45->setFrameOfReference(getFrame("I"));
  l45->setDirection("[1; 0; 0]");
  l45->setDiameter(7e-3);
  l45->setLength(.7);
  ZetaLinePressureLoss * pl_l45 = new ZetaLinePressureLoss();
  pl_l45->setZeta(19);
  l45->setLinePressureLoss(pl_l45);

  RigidLine * l56 = new RigidLine("l56");
  addObject(l56);
  l56->setFrameOfReference(getFrame("I"));
  l56->setDirection("[1; 0; 0]");
  l56->setDiameter(2e-3);
  l56->setLength(1.7);
  ZetaLinePressureLoss * pl_l56 = new ZetaLinePressureLoss();
  pl_l56->setZeta(10);
  l56->setLinePressureLoss(pl_l56);

  EnvironmentNode * n1 = new EnvironmentNode("n1");
  addLink(n1);
  n1->enableOpenMBV(.025, 0, 10e5, "[-.1; 0; 0]");
  n1->addOutFlow(l12a);
  n1->addOutFlow(l12b);
  n1->addOutFlow(l15);
  n1->addOutFlow(l16);

  ConstrainedNode * n2 = new ConstrainedNode("n2");
  addLink(n2);
  n2->enableOpenMBV(.025, 0, 10e5, "[-.05; .1; 0]");
  n2->setpFunction(new ConstantFunction1<double, double>(5e5));
  n2->addInFlow(l12a);
  n2->addInFlow(l12b);
  n2->addOutFlow(l23);

  HNode * n3;
  if (unilateral)
    n3 = new RigidNode("n3");
  else {
    ElasticNode * nTmp = new ElasticNode("n3");
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

  HNode * n4;
  if (unilateral)
    n4 = new RigidNode("n4");
  else {
    ElasticNode * nTmp = new ElasticNode("n4");
    nTmp->setVolume(3e-5);
    nTmp->setFracAir(0.01);
    nTmp->setp0(7e5);
    n4 = nTmp;
  }
  addLink(n4);
  n4->enableOpenMBV(.025, 0, 10e5, "[.1; 0; 0]");
  n4->addInFlow(l34);
  n4->addOutFlow(l45);

  HNode * n5;
  if (unilateral)
    n5 = new RigidNode("n5");
  else {
    ElasticNode * nTmp = new ElasticNode("n5");
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

  HNode * n6;
  if (unilateral)
    n6 = new RigidNode("n6");
  else {
    ElasticNode * nTmp = new ElasticNode("n6");
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

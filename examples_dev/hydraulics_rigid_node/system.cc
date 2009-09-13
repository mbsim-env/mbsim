#include "system.h"

#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/utils/function.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool setvalued) : Group(name) {

  RigidLine * l12 = new RigidLine("l12");
  addObject(l12);
  l12->setDiameter(5e-3);
  l12->setLength(1.7);
  ZetaLinePressureLoss * pl12 = new ZetaLinePressureLoss();
  pl12->setZeta(3);
  l12->setLinePressureLoss(pl12);
  l12->setFrameOfReference(getFrame("I"));
  l12->setDirection("[0;0;0]");

  RigidLine * l23 = new RigidLine("l23");
  addObject(l23);
  l23->setDiameter(7e-3);
  l23->setLength(.7);
  ZetaLinePressureLoss * pl23 = new ZetaLinePressureLoss();
  pl23->setZeta(1.5);
  l23->setLinePressureLoss(pl23);
  l23->setFrameOfReference(getFrame("I"));
  l23->setDirection("[0;0;0]");

  ConstrainedNode * n1 = new ConstrainedNode("n1");
  n1->setpFunction(new ConstantFunction1<double, double>(5e5));
  addLink(n1);
  n1->addOutFlow(l12);

  HNode * n2;
  if (setvalued)
    n2 = new RigidNode("n2");
  else {
    ElasticNode * nTmp = new ElasticNode("n2");
    nTmp->setVolume(5e-6);
    nTmp->setFracAir(0.03);
    nTmp->setp0(1e5);
    n2 = nTmp;
  }
  addLink(n2);
  n2->addInFlow(l12);
  n2->addOutFlow(l23);

  EnvironmentNode * n3 = new EnvironmentNode("n3");
  addLink(n3);
  n3->addInFlow(l23);

}

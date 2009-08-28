#include "system.h"

#include "mbsimHydraulics/hydline.h"
#include "mbsimHydraulics/hydnode.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/utils/function.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;

System::System(const string &name, bool setvalued) : Group(name) {

  HydLine * l12 = new HydLine("l12");
  addObject(l12);
  l12->setDiameter(5e-3);
  l12->setLength(1.7);
  l12->addPressureLoss(new PressureLossZeta("zeta1", 3));

  HydLine * l23 = new HydLine("l23");
  addObject(l23);
  l23->setDiameter(7e-3);
  l23->setLength(.7);
  l23->addPressureLoss(new PressureLossZeta("zeta1", 1.5));

  HydNodeConstrained * n1 = new HydNodeConstrained("n1");
  n1->setpFunction(new ConstantFunction1<double, double>(5e5));
  addLink(n1);
  n1->addOutFlow(l12);

  HydNode * n2;
  if (setvalued)
    n2 = new HydNodeRigid("n2");
  else {
    HydNodeElastic * nTmp = new HydNodeElastic("n2");
    nTmp->setVolume(5e-6);
    nTmp->setFracAir(0.03);
    nTmp->setp0(1e5);
    n2 = nTmp;
  }
  addLink(n2);
  n2->addInFlow(l12);
  n2->addOutFlow(l23);

  HydNodeEnvironment * n3 = new HydNodeEnvironment("n3");
  addLink(n3);
  n3->addInFlow(l23);

}

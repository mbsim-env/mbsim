#include "system.h"

#include "mbsimHydraulics/rigid_line.h"
#include "mbsimHydraulics/elastic_line_galerkin.h"
#include "mbsimHydraulics/hnode.h"
#include "mbsimHydraulics/pressure_loss.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/tabular_function.h"
#include "mbsim/functions/constant_function.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimHydraulics;

System::System(const string &name, bool setvalued) : Group(name) {

  ElasticLineGalerkin * l12 = new ElasticLineGalerkin("l12");
  addObject(l12);
  l12->setDiameter(6e-3);
  l12->setLength(0.7);
  l12->setp0(2e5);
  l12->setFracAir(0.03);
  l12->setdh(0);
  l12->setDLehr(0.02);
  l12->setAnsatzFunction(ElasticLineGalerkin::BSplineOrd3, 4);
  l12->setRelativePlotPoints(Vec("[0; .25; .5; .75; 1]"));
  l12->setFrameOfReference(getFrame("I"));
  l12->setDirection("[0;0;0]");

  RigidLine * l23 = new RigidLine("l23");
  addObject(l23);
  l23->setDiameter(4e-3);
  l23->setLength(.3);
  ZetaLinePressureLoss * zeta = new ZetaLinePressureLoss();
  zeta->setZeta(1.5);
  l23->setLinePressureLoss(zeta);
  l23->setFrameOfReference(getFrame("I"));
  l23->setDirection("[0;0;0]");

  ConstrainedNode * n1 = new ConstrainedNode("n1");
  n1->setpFunction(new ConstantFunction<double(double)>(3e5));
  addLink(n1);
  n1->addOutFlow(l12);

  HNode * n2;
  if (setvalued)
    n2 = new RigidNode("n2");
  else {
    ElasticNode * nTmp = new ElasticNode("n2");
    nTmp->setVolume(5e-6);
    nTmp->setFracAir(0.03);
    nTmp->setp0(2e5);
    n2 = nTmp;
  }
  addLink(n2);
  n2->addInFlow(l12);
  n2->addOutFlow(l23);

  EnvironmentNode * n3 = new EnvironmentNode("n3");
  addLink(n3);
  n3->addInFlow(l23);

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
  setPlotFeatureRecursive(volumeFlow, true);
  setPlotFeatureRecursive(pressure, true);
}

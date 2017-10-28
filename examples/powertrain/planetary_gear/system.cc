#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsimPowertrain/planetary_gear.h"
#include "mbsim/functions/constant_function.h"

#include "openmbvcppinterface/frustum.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimPowertrain;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  PlanetaryGear* planetaryGear = new PlanetaryGear("PlanetaryGear");
  addGroup(planetaryGear);

  KineticExcitation* ke;
  ke = new KineticExcitation("MS");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(planetaryGear->getObject("Sun"))->getFrame("C"));
  ke->setMomentDirection("[0;0;1]");
  ke->setMomentFunction(new ConstantFunction<VecV(double)>(0.1));

//  ke = new KineticExcitation("MT");
//  addLink(ke);
//  ke->connect(static_cast<RigidBody*>(planetaryGear->getObject("Carrier"))->getFrame("C"));
//  ke->setMoment("[0;0;1]", new Moment(-0.3));
//
//  ke = new KineticExcitation("MH");
//  addLink(ke);
//  ke->connect(static_cast<RigidBody*>(planetaryGear->getObject("Annulus"))->getFrame("C"));
//  ke->setMoment("[0;0;1]", new Moment(.99*0.2));

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
}


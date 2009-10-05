#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/rigid_body.h"
#include "mbsimPowertrain/differential_gear.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public Function1<fmatvec::Vec, double> {
  double M0;
  public:
    Moment(double M0_) : M0(M0_) {}
    fmatvec::Vec operator()(const double& tVal, const void * =NULL) {
      Vec M(1);
      M(0) = M0;
      return M;
    };
};

Pendulum::Pendulum(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  DifferentialGear* differentialGear = new DifferentialGear("PlanetaryGear");
  addGroup(differentialGear);
  static_cast<RigidBody*>(differentialGear->getObject("Shaft1"))->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  static_cast<RigidBody*>(differentialGear->getObject("Planet"))->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  KineticExcitation* ke;
  ke = new KineticExcitation("MAn");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(differentialGear->getObject("Shaft1"))->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(1.2));

  ke = new KineticExcitation("MAbL");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(differentialGear->getObject("Shaft4"))->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(0.99));

  ke = new KineticExcitation("MAbR");
  addLink(ke);
  ke->connect(static_cast<RigidBody*>(differentialGear->getObject("Shaft5"))->getFrame("C"));
  ke->setMoment("[0;0;1]", new Moment(1));

}


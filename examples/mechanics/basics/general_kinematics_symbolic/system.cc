#include "system.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/observers/frame_observer.h"

#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cube.h"

using namespace MBSim;
using namespace casadi;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);

  // Parameter der Körper
  double m1 = 3;
  SymMat Theta1(3,EYE);
  double h1 = 0.5;
  double freq1 = M_PI;
  double freq2 = M_PI/3;
  double v0y = 1;

  RigidBody *body1 = new RigidBody("Box1");
  addObject(body1);

  body1->setMass(m1);
  body1->setInertiaTensor(Theta1);

  body1->setFrameOfReference(getFrame("I"));
  body1->setFrameForKinematics(body1->getFrame("C"));

  SX sq=SX::sym("q", 2);

  SX st=SX::sym("t");

  SX pos=SX::zeros(3);
  pos(0) = cos(SX(sq(0)));
  pos(1) = sin(SX(sq(0)));
  pos(2) = sq(1);
  
  SymbolicFunction<Vec3(VecV,double)> *position = new SymbolicFunction<Vec3(VecV,double)>(pos, sq, st);
  body1->setTranslation(position);
  
  body1->setGeneralizedInitialVelocity("[0;1]");

  body1->getFrame("C")->setPlotFeature("position",enabled);
  body1->getFrame("C")->setPlotFeature("orientation",enabled);
  body1->getFrame("C")->setPlotFeature("velocity",enabled);
  body1->getFrame("C")->setPlotFeature("angularVelocity",enabled);
  body1->getFrame("C")->setPlotFeature("acceleration",enabled);
  body1->getFrame("C")->setPlotFeature("angularAcceleration",enabled);

  FrameObserver *o = new FrameObserver("Observer");
  addObserver(o);
  o->setFrame(body1->getFrame("C"));
  o->enableOpenMBVVelocity();

  // ----------------------- Visualisierung in OpenMBV --------------------  
  std::shared_ptr<OpenMBV::Cube> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  cuboid->setLength(h1);
  cuboid->setDiffuseColor(240./360.,1,1);
  body1->setOpenMBVRigidBody(cuboid);

  getFrame("I")->enableOpenMBV();

  setPlotFeatureRecursive("generalizedPosition",enabled);
  setPlotFeatureRecursive("generalizedVelocity",enabled);
}

#include "system.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_function.h"
#include "mbsim/observers/kinematics_observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cube.h"
#endif

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
  pos[0] = cos(SX(sq[0]));
  pos[1] = sin(SX(sq[0]));
  pos[2] = sq[1]; 

  vector<SX> input1(2);
  input1[0] = sq;
  input1[1] = st;

  SXFunction spos(input1,pos);
  
  SymbolicFunction<Vec3(VecV,double)> *position = new SymbolicFunction<Vec3(VecV,double)>(spos);
  body1->setTranslation(position);
  
  body1->setInitialGeneralizedVelocity("[0;1]");

  body1->getFrame("C")->setPlotFeature(globalPosition,enabled);
  body1->getFrame("C")->setPlotFeature(globalVelocity,enabled);
  body1->getFrame("C")->setPlotFeature(globalAcceleration,enabled);

  AbsoluteKinematicsObserver *o = new AbsoluteKinematicsObserver("Observer");
  addObserver(o);
  o->setFrame(body1->getFrame("C"));
  o->enableOpenMBVVelocity();

#ifdef HAVE_OPENMBVCPPINTERFACE
  // ----------------------- Visualisierung in OpenMBV --------------------  
  boost::shared_ptr<OpenMBV::Cube> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  cuboid->setLength(h1);
  cuboid->setDiffuseColor(240./360.,1,1);
  body1->setOpenMBVRigidBody(cuboid);

  getFrame("I")->enableOpenMBV();

#endif

}

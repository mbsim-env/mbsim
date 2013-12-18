#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/frame.h"
#include "mbsim/environment.h"
#include "mbsim/utils/symbolic_function.h"
#include "mbsim/observers/frame_observer.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/arrow.h"
#include "openmbvcppinterface/cube.h"
#endif

using namespace MBSim;
using namespace CasADi;
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

  vector<SX> sq(2);
  sq[0] = SX("q1");
  sq[1] = SX("q2");

  vector<SX> st(1);
  st[0] = SX("t");

  vector<SX> pos(3);
  pos[0] = cos(sq[0]);
  pos[1] = sin(sq[0]);
  pos[2] = sq[1]; 

  vector<vector<SX> > input1(2);
  input1[0] = sq;
  input1[1] = st;

  SXFunction spos(input1,pos);
  
  SymbolicFunction2<Vec3,Vec,double> *position = new SymbolicFunction2<Vec3,Vec,double>(spos);
  GeneralTranslation *trans = new GeneralTranslation(2,position);
  body1->setTranslation(trans);
  
  body1->setInitialGeneralizedVelocity("[0;1]");

  body1->getFrame("C")->setPlotFeature(globalPosition,enabled);
  body1->getFrame("C")->setPlotFeature(globalVelocity,enabled);
  body1->getFrame("C")->setPlotFeature(globalAcceleration,enabled);

  FrameObserver *o = new FrameObserver("Observer");
  addObserver(o);
  o->setFrame(body1->getFrame("C"));
  OpenMBV::Arrow *arrow = new OpenMBV::Arrow;
  arrow->setReferencePoint(OpenMBV::Arrow::fromPoint);
  arrow->setStaticColor(0.5);
  o->setOpenMBVVelocityArrow(arrow);

#ifdef HAVE_OPENMBVCPPINTERFACE
  // ----------------------- Visualisierung in OpenMBV --------------------  
  OpenMBV::Cube *cuboid=new OpenMBV::Cube;
  cuboid->setLength(h1);
  cuboid->setStaticColor(0.0);
  body1->setOpenMBVRigidBody(cuboid);

  getFrame("I")->enableOpenMBV();

#endif

}

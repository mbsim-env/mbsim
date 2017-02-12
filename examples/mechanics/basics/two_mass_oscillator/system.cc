#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"

#include "openmbvcppinterface/cube.h"
#include "openmbvcppinterface/coilspring.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  /* Parameters of body 1 */
  //mass
  double m1 = 5;
  // mass moment of inertia
  SymMat Theta1(3,EYE);
  //height
  double h1 = 0.5;

  /*Parameters of body 2 (analog to body 1)*/
  //mass
  double m2 = 2;
  SymMat Theta2(3,EYE);
  double h2 = 0.2;

  /* Parameters of spring 1 */
  // translation stiffness
  double c1 = 1e3;
  // damping coefficient
  double d1 = 0;
  // relaxed length
  double l01 = 0.5;

  /*Parameters of spring 2 (analog to spring 1)*/
  // translation stiffness
  double c2 = 1e3;
  double d2 = 0;
  double l02 = 0.2;

  // ----------------------- Definition of body 1 --------------------
  // define a rigid body (the name is "Box1")
  RigidBody *box1 = new RigidBody("Box1");

  // add body to the system (it is a object)
  addObject(box1);

  // Set mass and moment of inertia
  box1->setMass(m1);
  box1->setInertiaTensor(Theta1);

  // Define kinematics
  // If not given otherwise the Center of Gravity is used as reference frame for the kinetics
  // - define the reference frame (all degrees of freedom are defined within this frame)
  //   --> every system has one unique frame "I" which is its world frame
  box1->setFrameOfReference(getFrameI());

  //  - enable the body movement:
  //    the body can move translational along the second direction in the reference frame, i.e. y-direction of the "I"-frame of the system
  box1->setTranslation(new TranslationAlongYAxis<VecV>);
  //    REMARK: As no setRotation()-Routine is given the body can not rotate! (for rigid bodies the DoF have to be "unlocked")

  // ----- Define attachment points for the springs using additional frames on the body

  // Frame for lower-connection
  // Define relative position vector (half of the height in negative y-direction)
  Vec3 KrBLower;
  KrBLower(1) = - h1/2.;

  // Define a fixed relative frame
  FixedRelativeFrame * frameLower = new FixedRelativeFrame("LowerFrame");

  // set reference frame of the new frame (Frame C of box 1)
  frameLower->setFrameOfReference(box1->getFrameC());

  // set relative position of this frame
  frameLower->setRelativePosition(KrBLower);
  // REMARK: noe relative Rotation is given, thus the direction of the frame is the same as the reference-frame orientation

  // add frame to box 1
  box1->addFrame(frameLower);

  // Frame for upper connection (analog)
  Vec3 KrBUpper;
  KrBUpper(1) = h1/2.;
  FixedRelativeFrame * frameUpper = new FixedRelativeFrame("UpperFrame");
  frameUpper->setFrameOfReference(box1->getFrameC());
  frameUpper->setRelativePosition(KrBUpper);
  box1->addFrame(frameUpper);


  // ----------------------- Definition of body 2 (analog to body 1) --------------------
  RigidBody *box2 = new RigidBody("Box2");
  addObject(box2);
  box2->setMass(m2);
  box2->setInertiaTensor(Theta2);
  box2->setFrameOfReference(getFrameI());
  box2->setTranslation(new TranslationAlongYAxis<VecV>);
  Vec3 KrBLowerB2;
  KrBLowerB2(1) = - h2/2.;
  FixedRelativeFrame * frameLowerB2 = new FixedRelativeFrame("LowerFrame");
  frameLowerB2->setFrameOfReference(box2->getFrameC());
  frameLowerB2->setRelativePosition(KrBLowerB2);
  box2->addFrame(frameLowerB2);

  // ----------------------- Definition of spring 1 --------------------
  // create first spring
  SpringDamper *spring1 = new SpringDamper("Spring1");

  //add spring to the system
  addLink(spring1);

  // define the Force function (a linear force function with stiffness, damping value and relaxed length)
  spring1->setForceFunction(new LinearSpringDamperForce(c1,d1));
  spring1->setUnloadedLength(l01);

  // Define the two coordinate systems (i.e. frames) where the force law should act
  // --> (in this case between the lower frame on box1 and the I-Frame of the dynamic system)
  spring1->connect(box1->getFrame("LowerFrame"),getFrameI());

  // ----------------------- Definition of spring 2 (analog to spring 1) --------------------
  SpringDamper *spring2 = new SpringDamper("Spring2");
  addLink(spring2);
  spring2->setForceFunction(new LinearSpringDamperForce(c2,d2));
  spring2->setUnloadedLength(l02);
  spring2->connect(box1->getFrame("UpperFrame"),box2->getFrame("LowerFrame"));

  // ----------------------- Define initial states of the bodies -------------------
  // define initial generalized position of box1
  // Create vector for generalized positions:
  // --> Only one degree of freedom (linear-translation in y-direction)
  // --> Vector of length 1
  Vec q01(1, NONINIT);

  // set value of degree of freedom (y-position of CoG of box1)
  // at the end of the relaxed state of spring1 + half h1 + some "stretching of the first spring)
  q01(0) = l01 + h1/2 + 0.02;

  // set the position
  box1->setGeneralizedInitialPosition(q01);

  // define initial generalized position of box2 (directly)
  box2->setGeneralizedInitialPosition(Vec(1,INIT,l01 + l02 + h1 + h2/2));

  // ----------------------- Visualization in OpenMBV --------------------
  std::shared_ptr<OpenMBV::Cube> cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  cuboid->setLength(h1);
  cuboid->setDiffuseColor(1,0.5,0);
  cuboid->setTransparency(0.7);
  box1->setOpenMBVRigidBody(cuboid);

  cuboid=OpenMBV::ObjectFactory::create<OpenMBV::Cube>();
  cuboid->setLength(h2);
  cuboid->setDiffuseColor(0.5,1,1);
  cuboid->setTransparency(0.7);
  box2->setOpenMBVRigidBody(cuboid);

  spring1->enableOpenMBV(_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);

  spring2->enableOpenMBV(_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);

}


#include "system.h"

#include <fmatvec.h>

#include <mbsim/environment.h>
#include <mbsim/rigid_body.h>
#include <mbsim/contours/sphere.h>
#include <mbsim/contours/rectangle.h>
#include <mbsim/joint.h>
#include <mbsim/contact.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/kinematics.h>
#include "mbsim/utils/symbolic_function.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;
using namespace CasADi;

System::System(const string &projectName, int elements) :
    DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /*Upper Area*/
  Vec3 rArea;
  rArea(0) = 0;
  FixedRelativeFrame * refBoden = new FixedRelativeFrame("Boden - Referenz", rArea, BasicRotAIKz(M_PI_2));
  addFrame(refBoden);
  Rectangle * upperTable = new Rectangle("Boden");
  upperTable->setYLength(5e-2);
  upperTable->setZLength(1e-1);
  upperTable->setFrameOfReference(refBoden);
  upperTable->enableOpenMBV(true, 100);
  addContour(upperTable);

  /*Prepare Contact*/
  Contact* ballBoden = new Contact("Ball-Boden");
  if (not ODE) {
    ballBoden->setContactForceLaw(new UnilateralConstraint());
    ballBoden->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  }
  else {
    ballBoden->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5, 100)));
  }
  addLink(ballBoden);

  /*Balls*/

  SymMat3 theta;
  theta(0, 0) = 2. / 5. * (radius * radius) * mass / 10.;
  theta(1, 1) = theta(0, 0);
  theta(2, 2) = theta(1, 1);

  for (int i = 0; i < elements; i++) {
    balls.push_back(new RigidBody("Ball" + numtostr(i)));
    balls[i]->setMass(mass);
    balls[i]->setInertiaTensor(theta);
    balls[i]->setRotation(new RotationAboutAxesYZ());
    addObject(balls[i]);

    //Add Contour (+ visualisation)
    Sphere * sphere = new Sphere("Kugel");
    sphere->setFrameOfReference(balls[i]->getFrameC());
    sphere->setRadius(radius);
    sphere->enableOpenMBV(true);
    balls[i]->addContour(sphere);

    //add contact
    ballBoden->connect(upperTable, sphere);

    //Add Frame for next body
    Vec3 r;
    r(0) = distance;
    FixedRelativeFrame * refFrame = new FixedRelativeFrame("Ref", r);
    balls[i]->addFrame(refFrame);

  }

  balls[0]->setTranslation(new LinearTranslation(Mat3x3(EYE)));
  balls[0]->setFrameOfReference(getFrameI());

  for (int i = 1; i < elements; i++) {
    balls[i]->setFrameOfReference(balls[i - 1]->getFrame("Ref"));
  }

  /*Initialize position*/

  //initialize first straight part
  Vec q0(6, INIT, 0.);
  q0(0) = 0;
  q0(1) = radius;
  q0(3) = M_PI / 10.;
  balls[0]->setInitialGeneralizedPosition(q0);

  //initialize circle
  for (int ele = 1; ele < elements; ele++) {
    Vec3 abc;
    abc(0) = M_PI / 10.;
    balls[ele]->setInitialGeneralizedPosition(abc);
  }
}

void System::addTrajectory() {

  RigidBody* leader = new RigidBody("Leader");
  leader->setMass(1e-8);
  leader->setInertiaTensor(1e-8 * SymMat3(EYE));
  leader->setFrameOfReference(getFrameI());
  leader->setFrameForKinematics(leader->getFrameC());
  addObject(leader);

  leader->getFrame("C")->enableOpenMBV(0.01);

  SX t("t");
  vector<SX> fexp(3);
  double v = 1;
  double h = 3e-2;
  double x0 = 2e-2;
  double a = -h / (x0 * x0);
  double b = -2 * a * x0;
  fexp[0] = v * t;
  fexp[1] = radius + (a * v * t * v * t + b * v * t);
  fexp[2] = 0;
  SXFunction foo(t, fexp);

  SymbolicFunction1<Vec3, double> *f = new SymbolicFunction1<Vec3, double>(foo);

  leader->setTranslation(new TimeDependentTranslation(f));

  Joint* joint = new Joint("Joint");
  joint->setForceDirection(Mat3x3(EYE));
  if (not ODE) {
    joint->setForceLaw(new BilateralConstraint());
    joint->setImpactForceLaw(new BilateralImpact());
  }
  else {
    joint->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(1e7,100)));
  }

  joint->connect(leader->getFrameC(), balls[0]->getFrameC());

  addLink(joint);

}


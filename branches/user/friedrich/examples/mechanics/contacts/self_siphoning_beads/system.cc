#include "system.h"

#include <fmatvec/fmatvec.h>

#include <mbsim/environment.h>
#include <mbsim/rigid_body.h>
#include <mbsim/contours/sphere.h>
#include <mbsim/contours/rectangle.h>
#include <mbsim/joint.h>
#include <mbsim/isotropic_rotational_spring_damper.h>
#include <mbsim/contact.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include "mbsim/utils/symbolic_function.h"
#include "mbsim/utils/function_library.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;
using namespace CasADi;

System::System(const string &projectName, int elements) :
    DynamicSystemSolver(projectName), elements(elements) {
  if (elements) {
    Vec grav(3);
    grav(1) = -9.81;
    MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

    /*Upper Area*/
    Vec3 rArea;
    rArea(0) = 0;
    FixedRelativeFrame * refBoden = new FixedRelativeFrame("Oben", rArea, BasicRotAIKz(M_PI_2));
    addFrame(refBoden);
    Rectangle * upperTable = new Rectangle("Oben");
    upperTable->setYLength(3e-2);
    upperTable->setZLength(1e-1);
    upperTable->setFrameOfReference(refBoden);
    upperTable->enableOpenMBV(true, 100);
    addContour(upperTable);

    /*Lower Plane*/
    Vec3 rPlane;
    rPlane(1) = -8e-1;
    FixedRelativeFrame * refUnten = new FixedRelativeFrame("Unten - Referenz", rPlane, BasicRotAIKz(M_PI_2));
    addFrame(refUnten);
    Plane * lowerTable = new Plane("Unten");
    lowerTable->setFrameOfReference(refUnten);
    lowerTable->enableOpenMBV(true, 0.1);
    addContour(lowerTable);

    /*Prepare Contact*/
    Contact* ballOben = new Contact("Ball_Boden");
    if (not ODE) {
      ballOben->setNormalForceLaw(new UnilateralConstraint());
      ballOben->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    else {
      ballOben->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(stiffness, damping)));
    }
    addLink(ballOben);

    Contact* ballUnten = new Contact("Ball_Unten");
    if (not ODE) {
      ballUnten->setNormalForceLaw(new UnilateralConstraint());
      ballUnten->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    else {
      ballUnten->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(stiffness, 100*damping)));
    }
    addLink(ballUnten);

    /*Balls*/

    SymMat3 theta;
    theta(0, 0) = 2. / 5. * (radius * radius) * mass;
    theta(1, 1) = theta(0, 0);
    theta(2, 2) = theta(1, 1);

    for (int ele = 0; ele < elements; ele++) {
      balls.push_back(new RigidBody("Ball" + numtostr(ele)));
      balls[ele]->setMass(mass);
      balls[ele]->setInertiaTensor(theta);
      balls[ele]->setRotation(new RotationAboutAxesYZ<VecV>);
      addObject(balls[ele]);

      //Add Contour (+ visualisation)
      Sphere * sphere = new Sphere("Kugel");
      sphere->setFrameOfReference(balls[ele]->getFrameC());
      sphere->setRadius(radius);
      sphere->enableOpenMBV(true);
      balls[ele]->addContour(sphere);

      //add contact
      ballOben->connect(upperTable, sphere);

      ballUnten->connect(lowerTable, sphere);

      //Add Frame for next body
      Vec3 r;
      r(0) = distance;
      FixedRelativeFrame * refFrame = new FixedRelativeFrame("Ref", r);
      balls[ele]->addFrame(refFrame);

      if (ele > 1) {
        IsotropicRotationalSpringDamper * iso = new IsotropicRotationalSpringDamper("Iso" + numtostr(ele));
        iso->setParameters(0, 1e-7, 0);
        iso->setMomentDirection(Mat("[0,0;1,0;0,1]"));
        iso->connect(balls[ele - 1]->getFrameC(), balls[ele]->getFrameC());
        addLink(iso);
      }

    }

    balls[0]->setTranslation(new TranslationAlongAxesXYZ<VecV>);
    balls[0]->setFrameOfReference(getFrameI());

    for (int i = 1; i < elements; i++) {
      balls[i]->setFrameOfReference(balls[i - 1]->getFrame("Ref"));
    }

    /*Initialize position*/

    //initialize first straight part
    Vec q0(5, INIT, 0.);
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
}

void System::addTrajectory(double tEnd) {

  RigidBody* leader = new RigidBody("Leader");
  leader->setMass(1e-8);
  leader->setInertiaTensor(1e-8 * SymMat3(EYE));
  leader->setFrameOfReference(getFrameI());
  leader->setFrameForKinematics(leader->getFrameC());
  addObject(leader);

  leader->getFrame("C")->enableOpenMBV(0.01);

  SX t("t");
  vector<SX> fexp(3);
  double v = M_PI / tEnd;
  double x0 = 2e-2;
  fexp[0] = x0 * sin(v * t - (M_PI_2)) + x0;
  fexp[1] = radius + x0 * cos(v * t - (M_PI_2));
//  fexp[1] = radius + (a * v * t * v * t + b * v * t);
//  fexp[1] = radius;
  fexp[2] = 0;
  SXFunction foo(t, fexp);

  SymbolicFunction<Vec3(double)> *f = new SymbolicFunction<Vec3(double)>(foo);

  leader->setTranslation(f);
  if (elements) {
    Joint* joint = new Joint("Joint");
    joint->setForceDirection(Mat3x3(EYE));
    if (not ODE) {
      joint->setForceLaw(new BilateralConstraint());
      joint->setImpactForceLaw(new BilateralImpact());
    }
    else {
      joint->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(stiffness, damping)));
    }

    joint->connect(leader->getFrameC(), balls[0]->getFrameC());

    addLink(joint);
  }

}


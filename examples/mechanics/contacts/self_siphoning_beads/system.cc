#include "system.h"

#include <fmatvec/fmatvec.h>

#include <mbsim/environment.h>
#include <mbsim/frames/fixed_relative_frame.h>
#include <mbsim/objects/rigid_body.h>
#include <mbsim/contours/frustum.h>
#include <mbsim/contours/sphere.h>
#include <mbsim/contours/plate.h>
#include <mbsim/links/joint.h>
#include <mbsim/isotropic_rotational_links/spring_damper.h>
#include <mbsim/links/contact.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include "mbsim/functions/symbolic_functions.h"
#include "mbsim/functions/kinetic_functions.h"
#include "mbsim/functions/kinematic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/invisiblebody.h>
#include "openmbvcppinterface/frustum.h"
#include "openmbvcppinterface/sphere.h"
#include "openmbvcppinterface/ivbody.h"
#endif

using namespace std;
using namespace MBSim;
using namespace fmatvec;
using namespace casadi;

double createAngle(double i) {
  return M_PI / 200 * sin(i);
}

SelfSiphoningBeats::SelfSiphoningBeats(const string &projectName, int elements, double isoDamping) :
    DynamicSystemSolver(projectName), elements(elements) {
  if (elements) {
    const double R = 1.0 * (radius + distance / 2. / sin(angle / 2));

    Vec grav(3);
    grav(1) = -9.81;
    MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

    /*Upper Area*/
    Vec3 rArea;
    rArea(0) = 0;
    FixedRelativeFrame * refBoden = new FixedRelativeFrame("Oben", rArea, BasicRotAIKz(M_PI_2));
    addFrame(refBoden);
    Plate * upperTable = new Plate("Oben");
    upperTable->setYLength(3e-2);
    upperTable->setZLength(1e-1);
    upperTable->setFrameOfReference(refBoden);
    upperTable->enableOpenMBV(true, 100);
    addContour(upperTable);

    Vec3 rCup;
    rCup(0) = -1.5 * radius;
    rCup(2) = -R + radius;
    FixedRelativeFrame * refCup = new FixedRelativeFrame("CupFrame", rCup, BasicRotAIKz(0));
    addFrame(refCup);

    Frustum * cup = new Frustum("Becher");
    cup->setFrameOfReference(refCup);
    Vec2 radii(INIT, R);
    cup->setRadii(radii);
    cup->setHeight(R);
    cup->enableOpenMBV(_transparency = 0.5);
    cup->setOutCont(false);
    addContour(cup);

    /*Lower Plane*/
    Vec3 rPlane;
    rPlane(1) = -1e-1;
    FixedRelativeFrame * refUnten = new FixedRelativeFrame("Table - Reference", rPlane, BasicRotAIKz(M_PI_2));
    addFrame(refUnten);
    RigidBody* table = new RigidBody("Table");
    addObject(table);

    table->setFrameOfReference(refUnten);
    table->setMass(1.);
    table->setInertiaTensor(SymMat3(EYE));

    Plane * lowerTable = new Plane("Unten");
    lowerTable->setFrameOfReference(table->getFrameC());
    table->addContour(lowerTable);

#ifdef HAVE_OPENMBVCPPINTERFACE

    boost::shared_ptr<OpenMBV::IvBody> v_table = OpenMBV::ObjectFactory::create<OpenMBV::IvBody>();
    v_table->setIvFileName("objects/tisch.wrl");
    v_table->setScaleFactor(0.004);
    v_table->setDiffuseColor(Vec3(INIT,1.));
    v_table->setName("Table");
    v_table->setInitialRotation(0.,M_PI_2,M_PI_2);
    v_table->setInitialTranslation(0.,-0.1,0);

    table->setOpenMBVRigidBody(v_table);
#endif

    /*Prepare Contact*/
    Contact* ballOben = new Contact("Ball_Boden");
    if (not ODE) {
      ballOben->setNormalForceLaw(new UnilateralConstraint());
      ballOben->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    else {
      ballOben->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(stiffness, isoDamping)));
    }
    addLink(ballOben);

    Contact* ballUnten = new Contact("Ball_Unten");
    if (not ODE) {
      ballUnten->setNormalForceLaw(new UnilateralConstraint());
      ballUnten->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    else {
      ballUnten->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(stiffness, 100 * isoDamping)));
    }
    addLink(ballUnten);

    Contact* ballCup = new Contact("Ball_Cup");
    if (not ODE) {
      ballCup->setNormalForceLaw(new UnilateralConstraint());
      ballCup->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    else {
      ballCup->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(stiffness, 100 * isoDamping)));
    }
    addLink(ballCup);

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
      Vec3 color(INIT, 1.);
      color(0) = (double) (ele) / (elements - 1);
      sphere->enableOpenMBV(_diffuseColor = color);
      balls[ele]->addContour(sphere);

      //add contact
      ballOben->connect(upperTable, sphere);

      ballUnten->connect(lowerTable, sphere);

      ballCup->connect(cup, sphere);

      //Add Frame for next body
      Vec3 r;
      r(0) = distance;
      FixedRelativeFrame * refFrame = new FixedRelativeFrame("Ref", r);
      balls[ele]->addFrame(refFrame);

      if (ele > 1) {
        IsotropicRotationalSpringDamper * iso = new IsotropicRotationalSpringDamper("Iso" + numtostr(ele));
        iso->setParameters(0, isoDamping, 0);
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
    q0(1) = radius + elements * distance;
    q0(2) = -R;
    q0(4) = -M_PI_2;
    balls[0]->setInitialGeneralizedPosition(q0);

    //initialize circle
    double ang1 = createAngle(0);
    double ang2 = createAngle(0);
    for (int ele = 1; ele < elements; ele++) {
      Vec2 abc;
      abc(0) = ang1;
      abc(1) = ang2;
      if(ele%2) {
        ang1 = createAngle(ele);
        ang2 = createAngle(ele);
      }
      else {
        ang1 = -ang1;
        ang2 = -ang2;
      }
      balls[ele]->setInitialGeneralizedPosition(abc);
    }
  }
}

void SelfSiphoningBeats::addEmptyLeader() {
  RigidBody* leader = new RigidBody("Leader");
  leader->setMass(1e-8);
  leader->setInertiaTensor(1e-8 * SymMat3(EYE));
  leader->setFrameOfReference(getFrameI());
  leader->setFrameForKinematics(leader->getFrameC());
  addObject(leader);

  if (elements) {
    Joint* joint = new Joint("Joint");
    joint->setForceDirection(Mat3x3(EYE));
    joint->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(0, 0)));

    joint->connect(leader->getFrameC(), balls[0]->getFrameC());

    addLink(joint);
  }

}

void SelfSiphoningBeats::addTrajectory(double tEnd) {

  RigidBody* leader = new RigidBody("Leader");
  leader->setMass(1e-8);
  leader->setInertiaTensor(1e-8 * SymMat3(EYE));
  leader->setFrameOfReference(getFrameI());
  leader->setFrameForKinematics(leader->getFrameC());
  addObject(leader);

  leader->getFrame("C")->enableOpenMBV(0.01);

  SX t=SX::sym("t");
  SX fexp=SX::zeros(3);
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
    if (not ODE)
      joint->setForceLaw(new BilateralConstraint());
    else
      joint->setForceLaw(new RegularizedBilateralConstraint(new LinearRegularizedBilateralConstraint(stiffness, damping)));

    joint->connect(leader->getFrameC(), balls[0]->getFrameC());

    addLink(joint);
  }

}


#include "system.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/links/maxwell_contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/functions/kinematics/kinematics.h"

#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName, const int contactlaw, const int nB) : DynamicSystemSolver(projectName) {

  /* preliminaries */

  //balls
  vector<RigidBody*> balls;
  vector<Sphere*> spheres;
  double mass = 1;

  //contact
  double mu = 0.8;

  Vec3 WrOK;
  Vec KrKS(3);
  Vec KrKP(3);
  SymMat Theta(3);
  SqrMat AWK(3);

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  RigidBody* groundBase = new RigidBody("GroundBase");
  Frustum* ground = new Frustum("Ground");

  double h = 0.1; // height and offset of frustum
  double rI = 0.01; // inner radius
  double rO = 0.4; // outer radius
  ground->setOutCont(false);


  FixedRelativeFrame* frameK = new FixedRelativeFrame("K", WrOK, SqrMat3(EYE), getFrameI());
  addFrame(frameK);

  groundBase->setFrameOfReference(frameK);
  groundBase->setFrameForKinematics(groundBase->getFrame("C"));
  groundBase->setMass(1.);
  groundBase->setInertiaTensor(SymMat(3,EYE));
  this->addObject(groundBase);

  /*Add contour to ground base*/
  Vec radii(2,INIT,0);
  radii(0) = rI;
  radii(1) = rO;
  ground->setRadii(radii);
  ground->setHeight(h);
  ground->setFrameOfReference(groundBase->getFrameC());

  ground->enableOpenMBV();
  ground->getOpenMBVRigidBody()->setDrawMethod(OpenMBV::Body::lines);


  groundBase->addContour(ground);

  /* contact */
  Contact *contact;

  double stiffness = 1e5;
  double damping = 10000;
  double epsilon = damping * 1. / 10000;
  if (epsilon > 1)
    epsilon = 1;

  if(contactlaw == 0) { //Maxwell Contact
    contact = new MaxwellContact("Contact");
    //Normal force
    InfluenceFunction* infl = new FlexibilityInfluenceFunction(ground->getName(), 1./stiffness);
    static_cast<MaxwellContact*>(contact)->addContourCoupling(ground, ground, infl);
    static_cast<MaxwellContact*>(contact)->setDampingCoefficient(damping);

    //Frictional force
//    contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    contact->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    contact->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  }
  else if(contactlaw == 1) { //Regularized Unilateral Contact
    contact = new Contact("Contact");
    //Normal force
    contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,damping)));

    //Frictional force
//    contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    contact->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    contact->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  }
  else if (contactlaw == 2) { //Unilateral Constraint Contact
    contact = new Contact("Contact");
    //Normal force
    contact->setNormalForceLaw(new UnilateralConstraint);
    contact->setNormalImpactLaw(new UnilateralNewtonImpact(0));

    //Frictional force
    contact->setTangentialForceLaw(new SpatialCoulombFriction(mu));
    contact->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
  }
  //fancy stuff
  contact->enableOpenMBVContactPoints(0.01);
  contact->enableOpenMBVNormalForce(_scaleLength=0.001);
  contact->enableOpenMBVTangentialForce(_scaleLength=0.001);

  this->addLink(contact);

  // bodies
  for(int k=0; k<nB; k++) {
    stringstream name;
    name << "Ball_" << k;
    balls.push_back(new RigidBody(name.str()));
    this->addObject(balls[k]);

    stringstream frame; // ball location
    frame << "B_"  << k;
    Vec WrOS0B(3,INIT,0.);
    WrOS0B(0) = (rI+rO)*0.75*cos(k*2.*M_PI/nB);
    WrOS0B(1) = h;
    WrOS0B(2) = (rI+rO)*0.85*sin(k*2.*M_PI/nB);
    FixedRelativeFrame * refFrame = new FixedRelativeFrame(frame.str(),WrOS0B,SqrMat3(EYE),getFrameI());
    this->addFrame(refFrame);

    balls[k]->setFrameOfReference(refFrame);
    balls[k]->setFrameForKinematics(balls[k]->getFrame("C"));
    balls[k]->setMass(mass);
    balls[k]->setInertiaTensor(Theta);
    balls[k]->setTranslation(new TranslationAlongAxesXYZ<VecV>); // only translational dof because of point masses

    Vec u0(3,INIT,0);
    u0(1) = -1;
    balls[k]->setGeneralizedInitialVelocity(u0);

    stringstream spherename;
    spherename << "sphere_" << k;
    spheres.push_back(new Sphere(spherename.str()));
    spheres[k]->setRadius(0.01);
    spheres[k]->enableOpenMBV();
    spheres[k]->setFrameOfReference(balls[k]->getFrameC());

    balls[k]->addContour(spheres[k]);

    contact->connect(groundBase->getContour("Ground"),spheres[k]);
  }
}


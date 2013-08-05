#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contour.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#endif

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

#ifdef HAVE_OPENMBVCPPINTERFACE
  ground->enableOpenMBV();
  ground->getOpenMBVRigidBody()->setDrawMethod(OpenMBV::Body::lines);
#endif


  groundBase->addContour(ground);

  /* contact */
  Contact *contact = new Contact("Contact");

#ifdef HAVE_OPENMBVCPPINTERFACE
  /*Print arrows for contacts*/
  OpenMBV::Arrow *normalArrow = new OpenMBV::Arrow();
  normalArrow->setScaleLength(0.001);
  OpenMBV::Arrow *frArrow = new OpenMBV::Arrow();
  frArrow->setScaleLength(0.001);
  frArrow->setStaticColor(0.75);

  //fancy stuff
  contact->enableOpenMBVContactPoints(0.01);
  contact->setOpenMBVNormalForceArrow(normalArrow);
  contact->setOpenMBVFrictionArrow(frArrow);
#endif

  double stiffness = 1e5;
  double damping = 10000;
  double epsilon = damping * 1. / 10000;
  if (epsilon > 1)
    epsilon = 1;

  if(contactlaw == 0) { //Maxwell Contact
    //Normal force
    InfluenceFunction* infl = new FlexibilityInfluenceFunction(ground->getShortName(), 1/stiffness);
    MaxwellUnilateralConstraint* mfl = new MaxwellUnilateralConstraint(damping);
    mfl->addContourCoupling(ground, ground, infl);
    contact->setContactForceLaw(mfl);

    //Frictional force
//    contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  }
  else if(contactlaw == 1) { //Regularized Unilateral Contact
    //Normal force
    contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5,damping)));

    //Frictional force
//    contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  }
  else if (contactlaw == 2) { //Unilateral Constraint Contact
    //Normal force
    contact->setContactForceLaw(new UnilateralConstraint);
    contact->setContactImpactLaw(new UnilateralNewtonImpact(0));

    //Frictional force
    contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
    contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
  }
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
    balls[k]->setTranslation(new LinearTranslation(SqrMat(3,EYE))); // only translational dof because of point masses

    Vec u0(3,INIT,0);
    u0(1) = -1;
    balls[k]->setInitialGeneralizedVelocity(u0);

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


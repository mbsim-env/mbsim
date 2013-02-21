#include "system.h"

#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/sphere.h>
#endif

#include <mbsim/environment.h>
#include <mbsim/rigid_body.h>
#include <mbsim/contours/plane.h>
#include <mbsim/contours/planewithfrustum.h>
#include <mbsim/contours/point.h>
#include <mbsim/contours/circle_solid.h>
#include <mbsim/multi_contact.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/kinematics.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

/**
 * \brief Influence function that holds the analytical influence coefficients for a cantilever beam
 */
class CountourCouplingCantileverBeam : public InfluenceFunction {
  public:
    CountourCouplingCantileverBeam(double E_, double I_ ) :
        E(E_), I(I_) {
    }

    virtual ~CountourCouplingCantileverBeam() {
    }

    virtual double operator()(const fmatvec::Vec2 &Arg1, const fmatvec::Vec2 &Arg2, const void * = NULL) {
      double i=Arg1(0);  // it is: i < j
      double j=Arg2(0);
      if(i > j)
      {
        i = Arg2(0);
        j = Arg1(0);
      }

      return fabs((j*i*i - i*i*i/3) / (E*I*2));

    }

  protected:
    double E;
    double I;
};

System::System(const string &projectName, int contactType, int firstBall, int lastBall, const Vec& ReferenceFrameShift) :
    DynamicSystemSolver(projectName) {
  srand((unsigned) time(0));

  /*create reference frame for all objects with the given shift*/
  Frame* ReferenceFrame = new Frame(getName()+"RefFrame");
  this->addFrame(ReferenceFrame, ReferenceFrameShift, SqrMat(3,EYE));

  /*Print arrows for contacts*/
  OpenMBV::Arrow *normalArrow = new OpenMBV::Arrow();
  normalArrow->setScaleLength(0.00001);
  OpenMBV::Arrow *frArrow = new OpenMBV::Arrow();
  frArrow->setScaleLength(0.001);
  frArrow->setStaticColor(0.75);


  /*Parameters for the balls*/
  double space = 0.03;  //defines the space between two balls
  double radius = 0.01;
  SymMat ThetaBall(3, EYE);

  /*Parameters for the beam*/
  double length = 2*(space+ 2*radius) + (2*radius+space) * (1+lastBall) ;
  double depth = 2*radius;
  double height = depth;
  double E = 2.1e11; //Youngs modulus
  double I = depth * height * height * height / 12; //Area moment of inertia

  /*Parameters of contact*/
  double mu = 0.5; //friction coefficient

  /*Create Beam-body */
  RigidBody* Beam = new RigidBody("Beam");

  Beam->setFrameOfReference(ReferenceFrame);
  Beam->setMass(1);
  Beam->setInertiaTensor(SymMat(3,EYE));

  this->addObject(Beam);

  /*Add Contour to "Pyramid"*/

  Plane* BeamContour = new Plane("Line");

  //rotation of the plane contour on the upper side of the beam
  double planeRot = M_PI/2;

  //translation of the plane
  Vec planeTrans(3, INIT, 0.);
  planeTrans(1) = height/2;

  Beam->addContour(BeamContour, planeTrans, BasicRotAIKz(planeRot));

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid *openMBVBeam=new OpenMBV::Cuboid();
  openMBVBeam->setInitialTranslation(length/2,0,0);
  openMBVBeam->setLength(length,height,depth);
  openMBVBeam->setDrawMethod(OpenMBV::Body::DrawStyle(1));
  Beam->setOpenMBVRigidBody(openMBVBeam);
#endif

  /*Add Balls to System*/
  vector<RigidBody*> balls;
  vector<Point*> ballsContours;
  for (int ballIter = 0; ballIter < lastBall-firstBall+1; ballIter++) {

    //generate the name of each ball
    stringstream ballname;
    ballname << "Ball" << ballIter+firstBall;
    balls.push_back(new RigidBody(ballname.str()));

    //translation of the ball in longitudinal beam direction
    Vec BallInitialTranslation(3, INIT, 0.);

    BallInitialTranslation(0) = 2*(space+ 2*radius) + (ballIter+firstBall) * (space+ 2*radius);
    BallInitialTranslation(1) = height/2. + radius + 0.01;

    SqrMat BallRot(3, EYE);

    this->addFrame(ballname.str(), BallInitialTranslation, BallRot, ReferenceFrame);

    balls[ballIter]->setFrameOfReference(this->getFrame(ballname.str(), true));
    balls[ballIter]->setMass(1.);
    balls[ballIter]->setInertiaTensor(SymMat(3,EYE));
    balls[ballIter]->setTranslation(new LinearTranslation(SqrMat(3, EYE)));
    balls[ballIter]->setInitialGeneralizedVelocity(Vec("[0;-1;0]"));

    this->addObject(balls[ballIter]);

    /*Add (point-)contour of the balls*/
    ballname << "SolidContour";
    ballsContours.push_back(new Point(ballname.str()));

    SqrMat CircContourRot(3, EYE);
    Vec pointTrans = Vec(3, INIT, 0.);
    pointTrans(1) = - radius;

    balls[ballIter]->addContour(ballsContours[ballIter], pointTrans, CircContourRot);

    /*Visualization of the balls*/
#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Sphere *openMBVSphere=new OpenMBV::Sphere();
    openMBVSphere->setRadius(radius);

    switch (contactType) {
      case 0:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(0));
        openMBVSphere->setStaticColor(0.4);
        break;
      case 1:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(1));
        openMBVSphere->setStaticColor(0.6);
        break;
      case 2:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(1));
        openMBVSphere->setStaticColor(0.8);
        break;
    }
    balls[ballIter]->setOpenMBVRigidBody(openMBVSphere);
#endif

    //Add flexibility(stiffness) of balls
//    if (contactType == 0) {
//      FlexibilityInfluenceFunction* ballFlexibility= new FlexibilityInfluenceFunction(ballsContours[ballIter]->getName(), 1e-8);
//      maxwellContact->addContourCoupling(ballsContours[ballIter], ballsContours[ballIter], ballFlexibility);
//
//      //contour coupling between two neighbour-contours
//      if (ballIter % 2 == 1) {
//        ConstantInfluenceFunction* ballCoupling = new ConstantInfluenceFunction(ballsContours[ballIter - 1]->getName(), ballsContours[ballIter]->getName(), 1e-7);
//        maxwellContact->addContourCoupling(ballsContours[ballIter - 1], ballsContours[ballIter], ballCoupling);
//      }
//    }
  }

  Contact* contact = new Contact("Contact");

  //fancy stuff
  contact->enableOpenMBVContactPoints(1.,false);
  contact->setOpenMBVNormalForceArrow(normalArrow);
  contact->setOpenMBVFrictionForceArrow(frArrow);


  for (size_t contactIter = 0; contactIter < balls.size(); contactIter++) {
    stringstream contactname;
    contactname << "Contact_Beam_" << ballsContours[contactIter]->getName();

    contact->connect(BeamContour, ballsContours[contactIter]);
  }

  addLink(contact);

  switch (contactType) {
    case 0: //Maxwell Contact
    {
      MaxwellUnilateralConstraint* mcl = new MaxwellUnilateralConstraint();
      contact->setContactForceLaw(mcl);
      //Debug features
      mcl->setDebuglevel(0);

      CountourCouplingCantileverBeam* couplingBeam = new CountourCouplingCantileverBeam(E, I);
      mcl->addContourCoupling(BeamContour, BeamContour, couplingBeam);

      //Force Law (friction)
//      contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
      contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
      contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));

    }
    break;

    case 1: //regularized contact
    {
        double i = 2*(space+ 2*radius);  //this results in the stiffness of the first ball
        contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(3*E*I/ (i*i*i), 0)));

        //Force Law (friction)
        contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    }
    break;

    case 2:
    {
        contact->setContactForceLaw(new UnilateralConstraint);
        contact->setContactImpactLaw(new UnilateralNewtonImpact(1.));

        //Force Law (friction)
        contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
        contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));
    }
    break;
    default:
      throw MBSimError("No valid contactType chosen.");

  }

}


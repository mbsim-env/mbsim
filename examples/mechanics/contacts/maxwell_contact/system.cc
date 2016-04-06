#include "system.h"

#include <fmatvec/fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/sphere.h>
#endif

#include <mbsim/environment.h>
#include <mbsim/frames/fixed_relative_frame.h>
#include <mbsim/objects/rigid_body.h>
#include <mbsim/contours/plane.h>
#include <mbsim/contours/planewithfrustum.h>
#include <mbsim/contours/point.h>
#include <mbsim/links/contact.h>
#include <mbsim/links/maxwell_contact.h>
#include <mbsim/constitutive_laws/constitutive_laws.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/functions/kinetics/kinetics.h>
#include "mbsim/functions/kinematics/kinematics.h"

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

    virtual double operator()(const std::pair<Contour*, ContourFrame*>& firstContourInfo, const std::pair<Contour*, ContourFrame*>& secondContourInfo) {
      Vec2 Arg1 = evalZeta(firstContourInfo);
      Vec2 Arg2 = evalZeta(secondContourInfo);
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
  FixedRelativeFrame* ReferenceFrame = new FixedRelativeFrame(getName()+"RefFrame");
  ReferenceFrame->setRelativePosition(ReferenceFrameShift);
  this->addFrame(ReferenceFrame);

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

  Beam->addFrame(new FixedRelativeFrame("Line", planeTrans, BasicRotAIKz(planeRot)));
  BeamContour->setFrameOfReference(Beam->getFrame("Line"));
  Beam->addContour(BeamContour);

#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Cuboid> openMBVBeam=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
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

    FixedRelativeFrame * R = new FixedRelativeFrame(ballname.str());
    R->setFrameOfReference(ReferenceFrame);
    R->setRelativePosition(BallInitialTranslation);
    R->setRelativeOrientation(BallRot);

    this->addFrame(R);

    balls[ballIter]->setFrameOfReference(R);
    balls[ballIter]->setMass(1.);
    balls[ballIter]->setInertiaTensor(SymMat(3,EYE));
    balls[ballIter]->setTranslation(new TranslationAlongAxesXYZ<VecV>);
    balls[ballIter]->setInitialGeneralizedVelocity(Vec("[0;-1;0]"));

    this->addObject(balls[ballIter]);

    /*Add (point-)contour of the balls*/
    ballname << "SolidContour";
    ballsContours.push_back(new Point(ballname.str()));

    SqrMat CircContourRot(3, EYE);
    Vec pointTrans = Vec(3, INIT, 0.);
    pointTrans(1) = - radius;

    balls[ballIter]->addFrame(new FixedRelativeFrame(ballname.str(), pointTrans, CircContourRot));
    ballsContours[ballIter]->setFrameOfReference(balls[ballIter]->getFrame(ballname.str()));
    balls[ballIter]->addContour(ballsContours[ballIter]);

    /*Visualization of the balls*/
#ifdef HAVE_OPENMBVCPPINTERFACE
    boost::shared_ptr<OpenMBV::Sphere> openMBVSphere=OpenMBV::ObjectFactory::create<OpenMBV::Sphere>();
    openMBVSphere->setRadius(radius);

    switch (contactType) {
      case 0:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(0));
        openMBVSphere->setDiffuseColor(0.3333,1,0.6666);
        break;
      case 1:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(1));
        openMBVSphere->setDiffuseColor(0,1,0.6666);
        break;
      case 2:
        openMBVSphere->setDrawMethod(OpenMBV::Body::DrawStyle(1));
        openMBVSphere->setDiffuseColor(0.6666,1,1);
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

  Contact *contact;

  switch (contactType) {
    case 0: //Maxwell Contact
    {
      contact = new MaxwellContact("Contact");
      //Debug features
      //contact->setDebuglevel(0);

      CountourCouplingCantileverBeam* couplingBeam = new CountourCouplingCantileverBeam(E, I);
      static_cast<MaxwellContact*>(contact)->addContourCoupling(BeamContour, BeamContour, couplingBeam);

      //Force Law (friction)
//      contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
      contact->setTangentialForceLaw(new SpatialCoulombFriction(mu));
      contact->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
    }
    break;

    case 1: //regularized contact
    {
      contact = new Contact("Contact");
      double i = 2*(space+ 2*radius);  //this results in the stiffness of the first ball
      contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(3*E*I/ (i*i*i), 0)));
      contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));
    }
    break;

    case 2:
    {
      contact = new Contact("Contact");
      contact->setNormalForceLaw(new UnilateralConstraint);
      contact->setNormalImpactLaw(new UnilateralNewtonImpact(1.));
      contact->setTangentialForceLaw(new SpatialCoulombFriction(mu));
      contact->setTangentialImpactLaw(new SpatialCoulombImpact(mu));
    }
    break;
    default:
      THROW_MBSIMERROR("No valid contactType chosen.");
  }

  contact->enableOpenMBVContactPoints(1.,false);
  contact->enableOpenMBVNormalForce(_scaleLength=0.00001);
  contact->enableOpenMBVTangentialForce(_scaleLength=0.001);

  for (size_t contactIter = 0; contactIter < balls.size(); contactIter++) {
    stringstream contactname;
    contactname << "Contact_Beam_" << ballsContours[contactIter]->getName();

    contact->connect(BeamContour, ballsContours[contactIter]);
  }

  addLink(contact);

  //fancy stuff

}


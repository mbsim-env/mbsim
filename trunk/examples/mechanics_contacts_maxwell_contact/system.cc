#include "system.h"

#include <fmatvec.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/arrow.h>
#endif

#include <mbsim/environment.h>
#include <mbsim/maxwell_contact.h>
#include <mbsim/rigid_body.h>
#include <mbsim/contours/frustum.h>
#include <mbsim/contours/planewithfrustum.h>
#include <mbsim/contours/plane.h>
#include <mbsim/contours/circle_solid.h>
#include <mbsim/contact.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/rotarymatrices.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/kinematics.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

class CountourCouplingPyramid : public InfluenceFunction {
  public:
    CountourCouplingPyramid(const std::string& ContourName_) :
        InfluenceFunction(ContourName_, ContourName_) {
    }

    virtual ~CountourCouplingPyramid() {
    }

    double operator()(const fmatvec::Vec &Arg1, const fmatvec::Vec &Arg2, const void * = NULL) {
      if (fabs(Arg1(1) - Arg2(1)) < M_PI_2
      )
        return 1e-5 * pow(cos(Arg1(1) - Arg2(1)),2);
      else
        return 0;
    }
};

System::System(const string &projectName, int contactType, int circleNums) :
    DynamicSystemSolver(projectName) {
  srand((unsigned) time(0));

  /*Lemke Test*/
//  SqrMat M("[1,2,0; 0,1,2; 2,0,1]"); // matrix for a cycling problem
//  Vec q(3,INIT,-1.); //vector for a cycling problem
//
//
//
//  LemkeAlgorithm Lemke(M,q,true);
//
//  Vec solution = Lemke.solve();
//
//  cout << solution << endl;
//
//  return;


  /*add gravity*/
//  Vec grav(3);
//  grav(1) = -9.81;
//  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
  /*General-Parameters*/
  MaxwellContact *maxwellContact = new MaxwellContact("MaxwellContact");

  /*Print arrows for contacts*/
  OpenMBV::Arrow *normalArrow = new OpenMBV::Arrow();
  normalArrow->setScaleLength(0.001);
  OpenMBV::Arrow *frArrow = new OpenMBV::Arrow();
  frArrow->setScaleLength(0.001);

  /*Parameters for the "Pyramid"*/
  double massPyr = 1;
  double heightPyr = 0.5;
  Vec radiiPyr(2, INIT, 0.);
  radiiPyr(0) = 2;
  radiiPyr(1) = 1;
  SymMat ThetaPyr(3, EYE);

  /*Parameters for the "Circle"*/
  double massCirc = 1;
  double radiusCirc = 0.5;
  SymMat ThetaCirc(3, EYE);

  /*Parameters of contact*/
  double mu = 0.5; //friction coefficient

  /*Add plane to the world*/
//  Plane* PyramidContour = new Plane("Plane");
//  PyramidContour->enableOpenMBV();
//  this->addContour(PyramidContour, Vec(3, INIT, 0.), BasicRotAIKz(M_PI_2));
  /*Create Frustum-body "Pyramid"*/
  RigidBody* Pyramid = new RigidBody("Pyramid");

  Pyramid->setFrameOfReference(this->getFrameI());
  Pyramid->setMass(massPyr);
  Pyramid->setInertiaTensor(ThetaPyr);

  this->addObject(Pyramid);

  /*Add Contour to "Pyramid"*/

  Frustum* PyramidContour = new Frustum("Frustum");

  PyramidContour->setHeight(heightPyr);
  PyramidContour->setRadii(radiiPyr);
  PyramidContour->setOutCont(true);
  PyramidContour->enableOpenMBV();

  Pyramid->addContour(PyramidContour, Vec(3, INIT, 0.), SqrMat(3, EYE));

  /*Add Circles to System*/
  vector<RigidBody*> circles;
  vector<CircleSolid*> circleContours;
  for (int circIter = 0; circIter < circleNums; circIter++) {
    stringstream circlename;
    circlename << "Circle" << circIter;
    circles.push_back(new RigidBody(circlename.str()));

    Vec CircInitialTranslation(3, INIT, 0.);
    double circRadiusPosition = radiiPyr(0) + 0.5 * (radiiPyr(1) - radiiPyr(0));
//    double circAzimuthalPosition = (rand() % (int)(2*M_PI*1000)) / 1000.0; //
    double circAzimuthalPosition = circIter/100.;// circIter * M_PI / 60 + floor(circIter / 2) * M_PI / 100.;

    CircInitialTranslation(0) = circRadiusPosition * cos(circAzimuthalPosition);
    CircInitialTranslation(1) = radiusCirc + heightPyr / 2 + 0.06;// + 0.0001 * circIter;
    CircInitialTranslation(2) = circRadiusPosition * sin(circAzimuthalPosition);

    SqrMat CircRot(3, EYE);
    CircRot(0, 0) = cos(circAzimuthalPosition);
    CircRot(2, 2) = cos(circAzimuthalPosition);
    CircRot(0, 2) = -sin(circAzimuthalPosition);
    CircRot(2, 0) = sin(circAzimuthalPosition);

    this->addFrame(circlename.str(), CircInitialTranslation, CircRot);

    circles[circIter]->setFrameOfReference(this->getFrame(circlename.str(), true));
    circles[circIter]->setMass(massCirc);
    circles[circIter]->setInertiaTensor(ThetaCirc);
    circles[circIter]->setTranslation(new LinearTranslation(SqrMat(3, EYE)));
    circles[circIter]->setInitialGeneralizedVelocity(Vec("[0;-1;0]"));

    this->addObject(circles[circIter]);

    /*Add contour of the Circle*/
    circlename << "SolidContour";
    circleContours.push_back(new CircleSolid(circlename.str()));

    circleContours[circIter]->setOutCont(true);
    circleContours[circIter]->setRadius(radiusCirc);
    circleContours[circIter]->enableOpenMBV();

    SqrMat CircContourRot(3, EYE);
    double circContRotAngle = -M_PI_2 + M_PI; //M_PI_2 leads to x-direction points in -y-direction of the world system
    CircContourRot(0, 0) = cos(circContRotAngle);
    CircContourRot(0, 1) = -sin(circContRotAngle);
    CircContourRot(1, 0) = sin(circContRotAngle);
    CircContourRot(1, 1) = cos(circContRotAngle);

    circles[circIter]->addContour(circleContours[circIter], Vec(3, INIT, 0.), CircContourRot);

    //give circle contours flexibility(stiffness)
//    if (!regularizedContact) {
//      StiffnessInfluenceFuntion* circleStiffness = new StiffnessInfluenceFuntion(circleContours[circIter]->getName(), 1e-5);
//      maxwellContact->addContourCoupling(circleContours[circIter], circleContours[circIter], circleStiffness);
//    }
//
//    //contour coupling between two neighbour-contours
//    if (circIter % 2 == 1 and !regularizedContact) {
//      ConstantInfluenceFunction* CircleCoupling = new ConstantInfluenceFunction(circleContours[circIter - 1]->getName(), circleContours[circIter]->getName(), 1e-6);
//      maxwellContact->addContourCoupling(circleContours[circIter - 1], circleContours[circIter], CircleCoupling);
//    }
  }

  switch (contactType) {
    case 0: //maxwell Contact
      //plotting features
      maxwellContact->setINFO(false);
      maxwellContact->enableOpenMBVContactPoints(1.,false);
      maxwellContact->enableOpenMBVNormalForceArrow(normalArrow);
      maxwellContact->enableOpenMBVFrictionForceArrow(frArrow);

      if (1) {
        CountourCouplingPyramid* couplingPyr = new CountourCouplingPyramid(PyramidContour->getName());
        maxwellContact->addContourCoupling(PyramidContour, PyramidContour, couplingPyr);
      }
      else {
        StiffnessInfluenceFunction* couplingPyr = new StiffnessInfluenceFunction(PyramidContour->getName(), 1e-5);
        maxwellContact->addContourCoupling(PyramidContour, PyramidContour, couplingPyr);
      }

      for (size_t contactIter = 0; contactIter < circles.size(); contactIter++) {
        maxwellContact->add(PyramidContour, circleContours[contactIter]);

        if (contactIter % 2 < 2) {
          maxwellContact->setPlotContactPoint(contactIter, true);
        }
      }

      maxwellContact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));

      this->addLink(maxwellContact);
    break;

    case 1: //regularized contact
      for (size_t contactIter = 0; contactIter < circles.size(); contactIter++) {
        stringstream contactname;
        contactname << "Contact_Pyr-" << circleContours[contactIter]->getName();

        Contact* contact = new Contact(contactname.str());
        //Force law (normal direction)
        contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e5, 0)));

        //Force Law (friction)
        contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(mu)));

        contact->connect(PyramidContour, circleContours[contactIter]);

        //fancy stuff
        contact->enableOpenMBVContactPoints(1.,false);
        contact->setOpenMBVNormalForceArrow(normalArrow);
        contact->setOpenMBVFrictionArrow(frArrow);

        this->addLink(contact);
      }
    break;

    case 2:
      for (size_t contactIter = 0; contactIter < circles.size(); contactIter++) {
        stringstream contactname;
        contactname << "Contact_Pyr-" << circleContours[contactIter]->getName();

        Contact* contact = new Contact(contactname.str());
        //Force law (normal direction)
        contact->setContactForceLaw(new UnilateralConstraint);
        contact->setContactImpactLaw(new UnilateralNewtonImpact(1.));

        //Force Law (friction)
        contact->setFrictionForceLaw(new SpatialCoulombFriction(mu));
        contact->setFrictionImpactLaw(new SpatialCoulombImpact(mu));

        contact->connect(PyramidContour, circleContours[contactIter]);

        //fancy stuff
        contact->enableOpenMBVContactPoints(1.,false);
        contact->setOpenMBVNormalForceArrow(normalArrow);
        contact->setOpenMBVFrictionArrow(frArrow);

        this->addLink(contact);
      }
    break;
    default:
      throw MBSimError("No valid contactType chosen.");

  }

}


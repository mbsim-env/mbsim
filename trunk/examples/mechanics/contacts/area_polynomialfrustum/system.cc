#include "system.h"
#include <mbsim/utils/rotarymatrices.h>
#include "mbsim/rigid_body.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

#include <mbsim/utils/colors.h>

#include <fmatvec.h>

#include <mbsim/frame.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/ivbody.h>
#include "openmbvcppinterface/cube.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) :
    DynamicSystemSolver(projectName) {

  //add in a gravity in x direction
  Vec3 g;
  g(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);

  /*Polynomial Frustum initialisation*/

  //BODY
  RigidBody* polyfrustum = new RigidBody("PolynomialFrustum");

  polyfrustum->setRotation(new RotationAboutXAxis); //change from ZAxis, rotation,1 degree of freedom
  polyfrustum->setMass(1);
  polyfrustum->setInertiaTensor(SymMat3(EYE));
  FixedRelativeFrame* rotPoly = new FixedRelativeFrame("RotPoly", Vec3(), BasicRotAKIz(M_PI_2));
  addFrame(rotPoly);
  polyfrustum->setFrameOfReference(rotPoly);

  //Give degrees of freedom
  polyfrustum->setInitialGeneralizedPosition(Vec("[0]"));  //set position of the frustum,1 degree of freedom
  polyfrustum->setInitialGeneralizedVelocity(Vec("[0]"));  //change from(0,0,0,0,0,0), now we have rotating velocity,1 degree of freedom

  this->addObject(polyfrustum);

  //CONTOUR
  Vec polynomialparameters("[1;2;-1]");  //polynomial y=-x^2+2x+1
  PolynomialFrustum* polyfrustumcontour = new PolynomialFrustum("PolyFrustumContour", polynomialparameters);
  polyfrustumcontour->setHeight(1);

  polyfrustumcontour->enableOpenMBV();

  FixedRelativeFrame * polyFrustumFrame = new FixedRelativeFrame("RelPolyFrame", Vec3(), BasicRotAKIy(0));
  polyfrustum->addFrame(polyFrustumFrame);
  polyfrustumcontour->setFrameOfReference(polyFrustumFrame);
  polyfrustum->addContour(polyfrustumcontour);

  /*Rectangle initialisation*/

  //CONTOUR
  { // Rectangle1
    Rectangle* area = new Rectangle("AREA1");
    area->setYLength(1);
    area->setZLength(1);
    area->enableOpenMBV();

    //BODY
    RigidBody* areaBody = new RigidBody("RectangleBody1");
    areaBody->setMass(1);
    areaBody->setFrameOfReference(this->getFrameI());
    areaBody->setInertiaTensor(SymMat3(EYE));

    areaBody->setTranslation(new LinearTranslation(Mat3x3(EYE)));
    areaBody->setRotation(new CardanAngles);
    //give degrees of freedom
    areaBody->setInitialGeneralizedPosition(Vec("[-1.5;0.5;0;0;0;0]"));
    areaBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(areaBody);

    FixedRelativeFrame * areaFrame = new FixedRelativeFrame("RectangleFrame", Vec3(), BasicRotAKIx(M_2_PI));
    areaBody->addFrame(areaFrame);
    area->setFrameOfReference(areaFrame);
    areaBody->addContour(area);

    //Add contact between frustum and area
    Contact* contact = new Contact("FrustumRectangle1");
    contact->connect(area, polyfrustumcontour);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();

    //Set contact law
    contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6, 10000)));
    //contact->setContactImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

  { // Rectangle2
    Rectangle* area = new Rectangle("AREA2");
    area->setYLength(2);
    area->setZLength(2);
    area->enableOpenMBV();

    //BODY
    RigidBody* areaBody = new RigidBody("RectangleBody2");
    areaBody->setMass(1);
    areaBody->setFrameOfReference(this->getFrameI());
    areaBody->setInertiaTensor(SymMat3(EYE));

    areaBody->setTranslation(new LinearTranslation(Mat3x3(EYE)));
    areaBody->setRotation(new CardanAngles);
    //give degrees of freedom
    areaBody->setInitialGeneralizedPosition(Vec("[1.5;0.8;0;0;0;0]"));
    areaBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(areaBody);

    FixedRelativeFrame * areaFrame = new FixedRelativeFrame("RectangleFrame", Vec3(), BasicRotAKIz(M_PI));
    areaBody->addFrame(areaFrame);
    area->setFrameOfReference(areaFrame);
    areaBody->addContour(area);

    //Add contact between frustum and area
    Contact* contact = new Contact("FrustumRectangle2");
    contact->connect(area, polyfrustumcontour);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();

    //Set contact law
    //Set contact law
    contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6, 10000)));
    //contact->setContactImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

  { // Rectangle3
    Rectangle* area = new Rectangle("AREA3");
    area->setYLength(0.5);
    area->setZLength(1);
    area->enableOpenMBV();

    //BODY
    RigidBody* areaBody = new RigidBody("RectangleBody3");
    areaBody->setMass(1);
    areaBody->setFrameOfReference(this->getFrameI());
    areaBody->setInertiaTensor(SymMat3(EYE));

    areaBody->setTranslation(new LinearTranslation(Mat3x3(EYE)));
    areaBody->setRotation(new CardanAngles);
    //give degrees of freedom
    areaBody->setInitialGeneralizedPosition(Vec("[0.;-0.3;1.5;0;0;0]"));
    areaBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(areaBody);

    FixedRelativeFrame * areaFrame = new FixedRelativeFrame("RectangleFrame", Vec3(), BasicRotAKIz(M_PI) * BasicRotAIKy(M_PI_2));
    areaBody->addFrame(areaFrame);
    area->setFrameOfReference(areaFrame);
    areaBody->addContour(area);

    //Add contact between frustum and area
    Contact* contact = new Contact("FrustumRectangle3");
    contact->connect(area, polyfrustumcontour);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();

    //Set contact law
    //Set contact law
    contact->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e3, 10)));
    //contact->setContactImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setFrictionForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

}


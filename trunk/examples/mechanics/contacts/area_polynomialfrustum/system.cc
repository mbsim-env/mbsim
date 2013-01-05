#include "system.h"
#include <mbsim/utils/rotarymatrices.h>
#include "mbsim/rigid_body.h"
#include "mbsim/contours/area.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"

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
  g(0) = 9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);


  /*Polynomial Frustum initialisation*/

  //BODY
  RigidBody* polyfrustum = new RigidBody("PolynomialFrustum");

  polyfrustum->setRotation(new RotationAboutXAxis); //change from ZAxis, rotation,1 degree of freedom
  polyfrustum->setMass(1);
  polyfrustum->setInertiaTensor(SymMat3(EYE));
  polyfrustum->setFrameOfReference(this->getFrameI());

  //Give degrees of freedom
  polyfrustum->setInitialGeneralizedPosition(Vec("[0]"));
  polyfrustum->setInitialGeneralizedVelocity(Vec("[1]"));

  this->addObject(polyfrustum);

  //CONTOUR
  PolynomialFrustum* polyfrustumcontour = new PolynomialFrustum(
      "PolyFrustumContour");
  polyfrustumcontour->setHeight(1);

  vector<double> polynomialparameters;//polynomial y=-x^2+2x+1
  polynomialparameters.push_back(2);
  polynomialparameters.push_back(0);
  polynomialparameters.push_back(-1);

  polyfrustumcontour->setPolynomialParameters(polynomialparameters);
  polyfrustumcontour->enableOpenMBV();

  polyfrustum->addContour(polyfrustumcontour, Vec3("[0;0;0]"), BasicRotAKIy(0));



  /*Area initialisation*/

  //CONTOUR
  Area* area = new Area("AREA");
  area->setLimitY(0.5);
  area->setLimitZ(0.25);
  //area->enableOpenMBV();  //TODO: segmentation fault might appear for some versions of openmbv

  //BODY
  RigidBody* areapastedbody = new RigidBody("AreaPasteBody");
  areapastedbody->setMass(1);
  areapastedbody->setFrameOfReference(this->getFrameI());
  areapastedbody->setInertiaTensor(SymMat3(EYE));

  //Enable kinematics
  Mat3V translationDirections(3);
  //enable movement in x-direction (first column)
  translationDirections(0, 0) = 1;
  translationDirections(1, 0) = 0;
  translationDirections(2, 0) = 0;

  //enable movement in y-direction (second column)
  translationDirections(0, 1) = 0;
  translationDirections(1, 1) = 1;
  translationDirections(2, 1) = 0;

  //enable movement in z-direction (third column)
  translationDirections(0, 2) = 0;
  translationDirections(1, 2) = 0;
  translationDirections(2, 2) = 1;

  areapastedbody->setTranslation(new LinearTranslation(translationDirections));
  areapastedbody->setRotation(new CardanAngles);
  //give degrees of freedom
  areapastedbody->setInitialGeneralizedPosition(Vec("[-1;1;-1.5;0;0;0]"));
  areapastedbody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

  this->addObject(areapastedbody);

  areapastedbody->addContour(area, Vec3("[0;0;0]"), BasicRotAIKy(-M_PI_4));

  //Add contact between frustum and area
  Contact* contact = new Contact("FrustumArea");
  contact->connect(area, polyfrustumcontour);

  //Set contact law
  contact->setContactForceLaw(new UnilateralConstraint);
  contact->setContactImpactLaw(new UnilateralNewtonImpact(0));

  this->addLink(contact);



}


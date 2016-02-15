#include "system.h"
#include <mbsim/utils/rotarymatrices.h>
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"
#include <mbsim/utils/colors.h>
#include <fmatvec/fmatvec.h>
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

  polyfrustum->setRotation(new RotationAboutXAxis<VecV>); //change from ZAxis, rotation,1 degree of freedom
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
  Vec polynomialparameters("[1;-2;-1]");  //polynomial y=-x^2+2x+1
  PolynomialFrustum* polyfrustumcontour = new PolynomialFrustum("PolyFrustumContour" + name, polynomialparameters);
  polyfrustumcontour->setHeight(-1);

  polyfrustumcontour->enableOpenMBV();

  FixedRelativeFrame * polyFrustumFrame = new FixedRelativeFrame("RelPolyFrame", Vec3(), BasicRotAKIz(M_PI));
  polyfrustum->addFrame(polyFrustumFrame);
  polyfrustumcontour->setFrameOfReference(polyFrustumFrame);
  polyfrustum->addContour(polyfrustumcontour);

  /*Plate initialisation*/

  //CONTOUR
  { // Plate1
    Plate* plate = new Plate("AREA1");
    plate->setYLength(1);
    plate->setZLength(1);
    plate->enableOpenMBV();

    //BODY
    RigidBody* plateBody = new RigidBody("Plate");
    plateBody->setMass(1);
    plateBody->setFrameOfReference(this->getFrameI());
    plateBody->setInertiaTensor(SymMat3(EYE));

    plateBody->setTranslation(new TranslationAlongAxesXYZ<VecV>);
    plateBody->setRotation(new RotationAboutAxesXYZ<VecV>);
    //give degrees of freedom
    plateBody->setInitialGeneralizedPosition(Vec("[-1.5;0.5;0;0;0;0]"));
    plateBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(plateBody);

    FixedRelativeFrame * plateFrame = new FixedRelativeFrame("PlateFrame", Vec3(), BasicRotAKIx(M_2_PI));
    plateBody->addFrame(plateFrame);
    plate->setFrameOfReference(plateFrame);
    plateBody->addContour(plate);

    //Add contact between frustum and plate
    Contact* contact = new Contact("FrustumPlate1");
    contact->connect(polyfrustumcontour, plate);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();
    contact->enableOpenMBVNormalForce();
    contact->enableOpenMBVTangentialForce();

    //Set contact law
    contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6, 10000)));
    //contact->setNormalImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setTangentialImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

  { // Plate2
    Plate* plate = new Plate("AREA2");
    plate->setYLength(2);
    plate->setZLength(2);
    plate->enableOpenMBV();

    //BODY
    RigidBody* plateBody = new RigidBody("PlateBody2");
    plateBody->setMass(1);
    plateBody->setFrameOfReference(this->getFrameI());
    plateBody->setInertiaTensor(SymMat3(EYE));

    plateBody->setTranslation(new TranslationAlongAxesXYZ<VecV>);
    plateBody->setRotation(new RotationAboutAxesXYZ<VecV>);
    //give degrees of freedom
    plateBody->setInitialGeneralizedPosition(Vec("[1.5;0.8;0;0;0;0]"));
    plateBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(plateBody);

    FixedRelativeFrame * plateFrame = new FixedRelativeFrame("PlateFrame", Vec3(), BasicRotAKIz(M_PI));
    plateBody->addFrame(plateFrame);
    plate->setFrameOfReference(plateFrame);
    plateBody->addContour(plate);

    //Add contact between frustum and plate
    Contact* contact = new Contact("FrustumPlate2");
    contact->connect(polyfrustumcontour, plate);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();
    contact->enableOpenMBVNormalForce();
    contact->enableOpenMBVTangentialForce();

    //Set contact law
    //Set contact law
    contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6, 10000)));
    //contact->setNormalImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setTangentialImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

  { // Plate3
    Plate* plate = new Plate("AREA3");
    plate->setYLength(0.5);
    plate->setZLength(1);
    plate->enableOpenMBV();

    //BODY
    RigidBody* plateBody = new RigidBody("PlateBody3");
    plateBody->setMass(1);
    plateBody->setFrameOfReference(this->getFrameI());
    plateBody->setInertiaTensor(SymMat3(EYE));

    plateBody->setTranslation(new TranslationAlongAxesXYZ<VecV>);
    plateBody->setRotation(new RotationAboutAxesXYZ<VecV>);
    //give degrees of freedom
    plateBody->setInitialGeneralizedPosition(Vec("[0.;-0.3;1.5;0;0;0]"));
    plateBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(plateBody);

    FixedRelativeFrame * plateFrame = new FixedRelativeFrame("PlateFrame", Vec3(), BasicRotAKIz(M_PI) * BasicRotAIKy(M_PI_2));
    plateBody->addFrame(plateFrame);
    plate->setFrameOfReference(plateFrame);
    plateBody->addContour(plate);

    //Add contact between frustum and plate
    Contact* contact = new Contact("FrustumPlate3");
    contact->connect(polyfrustumcontour, plate);

    contact->setPlotFeature(openMBV, enabled);
    contact->enableOpenMBVContactPoints();
    contact->enableOpenMBVNormalForce();
    contact->enableOpenMBVTangentialForce();

    //Set contact law
    //Set contact law
    contact->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e3, 10)));
    //contact->setNormalImpactLaw(new UnilateralNewtonImpact(0.3));
    contact->setTangentialForceLaw(new RegularizedSpatialFriction(new LinearRegularizedCoulombFriction(0.5)));
    //contact->setTangentialImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

}


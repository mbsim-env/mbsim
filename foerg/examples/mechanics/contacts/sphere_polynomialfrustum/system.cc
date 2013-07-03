#include "system.h"
#include <mbsim/utils/rotarymatrices.h>
#include "mbsim/rigid_body.h"
#include "mbsim/contours/sphere.h"
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
#include <openmbvcppinterface/arrow.h>
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
  polyfrustum->setInitialGeneralizedVelocity(Vec("[10]"));  //change from(0,0,0,0,0,0), now we have rotating velocity,1 degree of freedom

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

  //Spheres
  int numSphere = 10;
  for(int i = 0; i < numSphere; i++) {
    stringstream name;
    name << "Sphere" << i+1;
    Sphere* sphere = new Sphere(name.str());
    sphere->setRadius(0.01 * i + 0.01);
    sphere->enableOpenMBV();

    //BODY
    RigidBody* sphereBody = new RigidBody("Body" + name.str());
    sphereBody->setMass(1);
    sphereBody->setFrameOfReference(this->getFrameI());
    sphereBody->setInertiaTensor(SymMat3(EYE));

    sphereBody->setTranslation(new LinearTranslation(Mat3x3(EYE)));
    sphereBody->setRotation(new CardanAngles);
    //give degrees of freedom
    Vec q0(6, INIT, 0);
    q0(1) = 0.5;
    double phi = (double) i / numSphere * 2 * M_PI;
    q0(0) = -1.5 * cos(phi);
    q0(2) = -1.5 * sin(phi);
    sphereBody->setInitialGeneralizedPosition(q0);
    sphereBody->setInitialGeneralizedVelocity(Vec("[0;0;0;0;0;0]"));

    this->addObject(sphereBody);

    FixedRelativeFrame * sphereFrame = new FixedRelativeFrame("Frame" + name.str(), Vec3(), BasicRotAKIx(M_2_PI));
    sphereBody->addFrame(sphereFrame);
    sphere->setFrameOfReference(sphereFrame);
    sphereBody->addContour(sphere);

    //Add contact between frustum and sphere
    Contact* contact = new Contact("Frustum-" + name.str());
    contact->connect(sphere, polyfrustumcontour);

    contact->setPlotFeature(openMBV, enabled);
#ifdef HAVE_OPENMBVCPPINTERFACE
    contact->enableOpenMBVContactPoints();
    OpenMBV::Arrow* arrow = new OpenMBV::Arrow();
    contact->setOpenMBVNormalForceArrow(arrow);
    contact->setOpenMBVFrictionArrow(arrow);
#endif

    //Set contact law
    contact->setContactForceLaw(new UnilateralConstraint);
    contact->setContactImpactLaw(new UnilateralNewtonImpact(0));
    contact->setFrictionForceLaw(new SpatialCoulombFriction(0.5));
    contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.5));

    this->addLink(contact);
  }

}


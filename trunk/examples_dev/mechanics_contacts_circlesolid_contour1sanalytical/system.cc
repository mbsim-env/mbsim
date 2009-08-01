#include "system.h"

#include "mbsim/rigid_body.h"
#include "mbsim/kinematics.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/utils/rotarymatrices.h"

#include "tools/rigid_contour_functions1s.h"

using namespace std;
using namespace MBSim;
using namespace fmatvec;


System::System(const string &name) : DynamicSystemSolver(name) {

  double m1=1.0;
  double l1=0.1;

  SymMat Theta(3);
  Theta(2,2) = 1./12.*m1*l1*l1;

  Mat YZ;
  ifstream file("contour.asc");
  file >> YZ;
  file.close();

  // Kontur der Nocke 
  Vec searchpoints(72,INIT,0);
  for (int j=0; j<searchpoints.size(); j++)
    searchpoints(j)=j/double((searchpoints.size()-1))*2.*M_PI;

  FuncCrPC *funcCamContour=new FuncCrPC();
  funcCamContour->setYZ(YZ);

  RigidBody * cam = new RigidBody("Cam");
  cam->setFrameOfReference(this->getFrame("I"));
  cam->setFrameForKinematics(cam->getFrame("C"));
  cam->setMass(m1);
  cam->setInertiaTensor(1000*Theta);
  cam->setRotation(new RotationAboutFixedAxis("[0;0;1]"));
  cam->setInitialGeneralizedVelocity(4.*M_PI);
  this->addObject(cam);

  Contour1sAnalytical * camContour = new Contour1sAnalytical("Contour");
  camContour->setUserFunction(funcCamContour);
  camContour->setAlphaStart(0.);
  camContour->setAlphaEnd(2.*M_PI);
  camContour->setNodes(searchpoints);
  cam->addContour(camContour, "[.003; .01; 0]", Cardan2AIK(-M_PI/2., 0, -M_PI/2. ));
  camContour->enableOpenMBV(true, .1);

  addFrame("I2", "[0.05; 0.09; 0.0]", SqrMat(3, EYE));

  RigidBody * roll = new RigidBody("Roll");
  roll->setFrameOfReference(this->getFrame("I2"));
  roll->setFrameForKinematics(roll->getFrame("C"));
  roll->setMass(m1*10);
  roll->setInertiaTensor(Theta);
  roll->setRotation(new RotationAboutFixedAxis("[0;0;1]"));
  roll->setTranslation(new LinearTranslation("[0;1;0]"));
  roll->setInitialGeneralizedVelocity("[0;1.0]");
  this->addObject(roll);

  CircleSolid * rollContour = new CircleSolid("Contour");
  rollContour->setRadius(.01);
  roll->addContour(rollContour, Vec(3, INIT, 0), SqrMat(3, EYE));
  rollContour->enableOpenMBV(true, .1);

  Contact * contactCamRoll = new Contact("Contact");
  contactCamRoll->setContactForceLaw(new LinearRegularizedUnilateralConstraint(1e6, 1e4));
  contactCamRoll->setFrictionForceLaw(new LinearRegularizedPlanarCoulombFriction(1.));
//  contactCamRoll->setContactForceLaw(new UnilateralConstraint);
//  contactCamRoll->setContactImpactLaw(new UnilateralNewtonImpact);
  contactCamRoll->connect(cam->getContour("Contour"), roll->getContour("Contour"));
  addLink(contactCamRoll);

  RigidBody * frameCam = new RigidBody("frameCam");
  frameCam->setFrameOfReference(this->getFrame("I"));
  frameCam->setFrameForKinematics(frameCam->getFrame("C"));
  frameCam->setMass(m1);
  frameCam->setInertiaTensor(Theta);
  this->addObject(frameCam);
  frameCam->getFrame("C")->enableOpenMBV(.1, .9);

  RigidBody * frameRoll = new RigidBody("frameRoll");
  frameRoll->setFrameOfReference(this->getFrame("I"));
  frameRoll->setFrameForKinematics(frameRoll->getFrame("C"));
  frameRoll->setMass(m1);
  frameRoll->setInertiaTensor(Theta);
  this->addObject(frameRoll);
  frameRoll->getFrame("C")->enableOpenMBV(.1, .9);
}

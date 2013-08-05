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

  bool rigid=true;

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

  //FuncCrPC *funcCamContour=new FuncCrPC();
  FuncCrPC_PlanePolar *funcCamContour=new FuncCrPC_PlanePolar();
  funcCamContour->setYZ(YZ);

  RigidBody * cam = new RigidBody("Cam");
  cam->setFrameOfReference(this->getFrame("I"));
  cam->setFrameForKinematics(cam->getFrame("C"));
  cam->setMass(m1);
  cam->setInertiaTensor(1000.*Theta);
  cam->setRotation(new RotationAboutFixedAxis("[0;0;1]"));
  cam->setInitialGeneralizedVelocity(4.*M_PI);
  this->addObject(cam);

  Contour1sAnalytical * camContour = new Contour1sAnalytical("Contour");
  camContour->setContourFunction1s(funcCamContour);
  camContour->setAlphaStart(0.);
  camContour->setAlphaEnd(2.*M_PI);
  camContour->setNodes(searchpoints);
  cam->addContour(camContour, "[.003; .01; 0]", Cardan2AIK(-M_PI/2., 0, -M_PI/2. ));
#ifdef HAVE_OPENMBVCPPINTERFACE
  camContour->enableOpenMBV();
#endif

  addFrame("I2", "[0.05; 0.09; 0.0]", SqrMat(3, EYE));

  RigidBody * roll = new RigidBody("Roll");
  roll->setFrameOfReference(this->getFrame("I2"));
  roll->setFrameForKinematics(roll->getFrame("C"));
  roll->setMass(m1);
  roll->setInertiaTensor(Theta);
  roll->setRotation(new RotationAboutFixedAxis("[0;0;1]"));
  roll->setTranslation(new LinearTranslation("[0;1;0]"));
  roll->setInitialGeneralizedVelocity("[0;1.0]");
  this->addObject(roll);

  CircleSolid * rollContour = new CircleSolid("Contour");
  rollContour->setRadius(.01);
  roll->addContour(rollContour, Vec(3, INIT, 0), SqrMat(3, EYE));
#ifdef HAVE_OPENMBVCPPINTERFACE
  rollContour->enableOpenMBV();
#endif

  Contact * contactCamRoll = new Contact("Contact");
  if (rigid) {
    contactCamRoll->setContactForceLaw(new UnilateralConstraint);
    contactCamRoll->setContactImpactLaw(new UnilateralNewtonImpact);
    contactCamRoll->setFrictionForceLaw(new PlanarCoulombFriction(.1));
    contactCamRoll->setFrictionImpactLaw(new PlanarCoulombImpact(.1));
  }
  else {
    contactCamRoll->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e6, 1e4)));
    contactCamRoll->setFrictionForceLaw(new RegularizedPlanarFriction(new LinearRegularizedCoulombFriction(1.)));
  }
  contactCamRoll->connect(cam->getContour("Contour"), roll->getContour("Contour"));
#ifdef HAVE_OPENMBVCPPINTERFACE
  contactCamRoll->enableOpenMBVContactPoints(.005);
#endif
  addLink(contactCamRoll);

}

#include "journal_bearing.h"
#include "cylindersolid_cylinderhollow_ehd.h"
#include "ehd_force_law.h"
#include "ehd_contact.h"

#include "mbsim/joint.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_functions.h"
#include "mbsim/functions/basic_functions.h"
#include "mbsim/functions/kinematic_functions.h"
#include <mbsim/functions/kinetic_functions.h>
// Beginning Contact
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/constitutive_laws.h"
// End Contact
#include "mbsim/environment.h"

#include "mbsim/kinetic_excitation.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h" // ball
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace MBSimEHD;
using namespace fmatvec;
using namespace casadi;
using namespace std;
using namespace boost;

class Moment : public MBSim::Function<fmatvec::VecV(double)> {
  public:
    fmatvec::VecV operator()(const double& t) {
      double t0 = 0.00;
      double t1 = 0.1;
      double M0 = 200;
      VecV M(1);
      if (t < t0)
        M(0) = t * M0 / t0;
      else if (t < t1)
        M(0) = M0 - (t - t0) / (t1 - t0) * M0;
      else
        M(0) = 0;

      M(0) = M0;
      return M;
    }
};

JournalBearingSystem::JournalBearingSystem(const string &projectName) :
    DynamicSystemSolver(projectName) {

  //Add gravity to the system
  Vec grav(3, INIT, 0.);
  grav(1) = -9.81; // in negative y-direction
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // geometrical characteristics
  //ROD
  double journal_radius = 22.5e-3;
  double journal_length = 140e-3;
  double journal_mass = 2; //0.038; // m1
  SymMat journal_inertia(3, INIT, 0.);
  journal_inertia(0, 0) = 1.; // DUMMY
  journal_inertia(1, 1) = 1.; // DUMMY
  journal_inertia(2, 2) = journal_mass / 12. * journal_length * journal_length; // J1

  //CRANK
  double clearing = 1e-3;
  double housing_radius = journal_radius + clearing;
  double housing_length = journal_length + 10e-3;
  double housing_mass = 1; //0.038; // m1
  SymMat housing_inertia(3, INIT, 0.);
  housing_inertia(0, 0) = 1.; // DUMMY
  housing_inertia(1, 1) = 1.; // DUMMY
  housing_inertia(2, 2) = housing_mass / 12. * housing_length * housing_length; // J1

  /*
   * JOURNAL
   */
  RigidBody *journal = new RigidBody("Journal");
  this->addObject(journal);

  // kinematics
  journal->setFrameOfReference(getFrameI());
  journal->setTranslation(new TranslationAlongAxesXY<VecV>());
  //TODO: spatial movement: journal->setTranslation(new TranslationAlongAxesXYZ<VecV>());

  bool fixedRotation = true;
  if (fixedRotation) {
    // Define kinematically the angle (omega is the angular velocity)
    double omega = 3.141592 * 100;
    SX t=SX::sym("t"); // symbolic variable for time
    SX angleExp = omega*t; // expression for the angle
    SXFunction angleFuncSX(t,angleExp); // SXfunction for the angle
    SymbolicFunction<double(double)> *angleFunc = new SymbolicFunction<double(double)>(angleFuncSX); // function for the angle
    journal->setTimeDependentRotation(new NestedFunction<RotMat3(double(double))>(new RotationAboutZAxis<double>, angleFunc));
  }
  else {
    // Alternatively: define a load that is applied to the crank
    journal->setRotation(new RotationAboutZAxis<VecV>());
    journal->setInitialGeneralizedVelocity(0.);
    KineticExcitation *load = new KineticExcitation("Motor");
    addLink(load);
    load->setMomentFunction(new Moment);
    load->setMomentDirection(Mat("[0;0;1]"));
    load->connect(journal->getFrameC());
  }

  //kinetics
  journal->setMass(journal_mass);
  journal->setInertiaTensor(journal_inertia);

  //Add contour to journal
  Vec3 cFruRefPos;
  cFruRefPos(2) = - journal_length /2.;
  FixedRelativeFrame* cFruRefFrame = new FixedRelativeFrame("Contour_Reference", cFruRefPos, BasicRotAIKx(M_PI_2));
  cFruRefFrame->setFrameOfReference(journal->getFrameC());
  journal->addFrame(cFruRefFrame);

  MBSim::Frustum * journal_cFru = new MBSim::Frustum("Journal_Outer");
  journal->addContour(journal_cFru);
  journal_cFru->setFrameOfReference(cFruRefFrame);
  journal_cFru->setRadii(Vec2(INIT, journal_radius)); //same radius -> makes frustum to cylinder
  journal_cFru->setHeight(journal_length);
  journal_cFru->setOutCont(true); //contact should be outside

  // visualisation journal
  journal->getFrameC()->enableOpenMBV(0.7e-1); //-CoG-frame
  cFruRefFrame->enableOpenMBV(0.3e-1);

  journal_cFru->enableOpenMBV(_diffuseColor = "[0.6;0.3;0.6]", _transparency = 0.3); //contour

  /* Housing */
  RigidBody *housing = new RigidBody("Housing");
  this->addObject(housing);

  //kinematics
  housing->setFrameOfReference(getFrameI());

  //TODO: enable some DOFs?
  if (false) {
    housing->setTranslation(new TranslationAlongAxesXY<VecV>());
    housing->setRotation(new RotationAboutZAxis<VecV>());
  }

  //kinetics
  housing->setMass(housing_mass);
  housing->setInertiaTensor(housing_inertia);

  // add neutral contour to the housing
  Vec3 chouFruRefPos;
  chouFruRefPos(2) = - housing_length /2.;
  FixedRelativeFrame* chouFruRefFrame = new FixedRelativeFrame("Contour_Reference", chouFruRefPos, BasicRotAIKx(M_PI_2));
  chouFruRefFrame->setFrameOfReference(housing->getFrameC());
  housing->addFrame(chouFruRefFrame);

  Frustum* housing_cFru = new Frustum("Housing_Inner");
  housing->addContour(housing_cFru);

  housing_cFru->setFrameOfReference(chouFruRefFrame);
  housing_cFru->setRadii(Vec2(INIT, housing_radius)); //same radius -> makes frustum to cylinder
  housing_cFru->setHeight(housing_length);
  housing_cFru->setOutCont(false); //contact should be inside

  // visualisation journal
  housing->getFrameC()->enableOpenMBV(0.7e-1); //-CoG-frame
  housing_cFru->enableOpenMBV(_diffuseColor = "[0.2;0.3;0.6]", _transparency = 0.3); //contour

  /* LINK-DEFINTIION */
  // Contact Between journal and housing (EHD-Contact)
  EHDContact * ctJouHou = new EHDContact("Contact_Journal_Housing");
  addLink(ctJouHou);

  //create lubricant
  Lubricant lub;
  lub = Lubricant(0.414, 839, 0.43, false, Lubricant::Roelands, Lubricant::DowsonHigginson); //lubT1
  lub = Lubricant(0.0109, 778, 0, false, Lubricant::constVisc, Lubricant::constDen);//lubT2

  // discretization
  EHDPressureElement ele("quad9", 4);
  ele.setLubricant(lub);

// Create computational mesh (half fluid domain)
  RowVec2 yb;
  yb(0) = 0;
  yb(1) = 2 * M_PI * housing_radius;
  RowVec2 zb;
  zb(0) = -housing_length / 2;
  zb(1) = housing_length / 2; //TODO: differently in matlab!!
  MatVx2 xb(2);
  xb.set(0, yb);
  xb.set(1, zb);

  EHDMesh * msh = new EHDMesh(ele, xb, VecInt("[20; 3]"));

  msh->Boundary(EHDMesh::dbc, EHDMesh::x2m);    // z = -L / 2
  msh->Boundary(EHDMesh::per1, EHDMesh::x1m);   // y = 0
  msh->Boundary(EHDMesh::per2, EHDMesh::x1p);   // y = 2 * pi * R2
  msh->FinishMesh();

//  EHDForceLaw *fL = new EHDForceLaw();
  ctJouHou->setNormalForceLaw(msh);
  ctJouHou->enableOpenMBVContactPoints(1e-5);
  ctJouHou->enableOpenMBVNormalForce(1e-6);

  //contact kinematics (delivers the info for the mesh)
  ContactKinematicsCylinderSolidCylinderHollowEHD *cK = new ContactKinematicsCylinderSolidCylinderHollowEHD();
  ctJouHou->connect(housing_cFru, journal_cFru, cK);

}


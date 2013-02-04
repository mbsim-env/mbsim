#include<config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_cosserat.h"
#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

#ifdef HAVE_NURBS
#define MY_PACKAGE_BUGREPORT PACKAGE_BUGREPORT
#define MY_PACKAGE_NAME PACKAGE_NAME
#define MY_PACKAGE_VERSION PACKAGE_VERSION
#define MY_PACKAGE_TARNAME PACKAGE_TARNAME
#define MY_PACKAGE_STRING PACKAGE_STRING
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "nurbs.h"
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "vector.h"
#define PACKAGE_BUGREPORT MY_PACKAGE_BUGREPORT
#define PACKAGE_NAME MY_PACKAGE_NAME
#define PACKAGE_VERSION MY_PACKAGE_VERSION
#define PACKAGE_TARNAME MY_PACKAGE_TARNAME
#define PACKAGE_STRING MY_PACKAGE_STRING

using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1sCosserat::FlexibleBody1sCosserat(const string &name, bool openStructure_) : FlexibleBodyContinuum<double> (name), cylinder(new CylinderFlexible("Cylinder")), top(new FlexibleBand("Top")), bottom(new FlexibleBand("Bottom")), left(new FlexibleBand("Left")), right(new FlexibleBand("Right")), neutralFibre(new Contour1sFlexible("NeutralFibre")), angle(new Cardan()),  Elements(0), rotationalElements(0), L(0.), l0(0.), E(0.), G(0.),A(0.), I1(0.), rho(0.), R1(0.), cEps0D(0.), cEps1D(0.), openStructure(openStructure_), initialised(false), bound_ang_start(3,INIT,0.), bound_ang_end(3,INIT,0.), bound_ang_vel_start(3,INIT,0.), bound_ang_vel_end(3,INIT,0.), cuboidBreadth(0.), cuboidHeight(0.), cylinderRadius(0.), curve(new NurbsCurve1s("Curve")){

  }

  FlexibleBody1sCosserat::~FlexibleBody1sCosserat(){}

  void FlexibleBody1sCosserat::updateh(double t, int k) {
    /* translational elements */
    FlexibleBodyContinuum<double>::updateh(t);

    /* rotational elements */
    for(int i=0;i<(int)rotationDiscretization.size();i++) {
      try { rotationDiscretization[i]->computeh(qRotationElement[i],uRotationElement[i]); } // compute attributes of finite element
      catch(MBSimError error) { error.printExceptionMessage(); throw; }
    }
    for(int i=0;i<(int)rotationDiscretization.size();i++) GlobalVectorContributionRotation(i,rotationDiscretization[i]->geth(),h[0]); // assemble
  }

  void FlexibleBody1sCosserat::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<double>::updateStateDependentVariables(t);

#ifdef HAVE_NURBS
    curve->computeCurveTranslations();
    curve->computeCurveVelocities();
    curve->computeCurveJacobians();
#endif
  }
}

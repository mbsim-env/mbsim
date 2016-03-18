#include <config.h>
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_cosserat.h"
#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include <mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_cosserat.h>
#include "mbsimFlexibleBody/utils/cardan.h"
#include "mbsim/dynamic_system_solver.h"
#include <mbsim/environment.h>
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include <mbsimFlexibleBody/discretization_interface.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  FlexibleBody1sCosserat::FlexibleBody1sCosserat(const string &name, bool openStructure) : FlexibleBody1s(name,openStructure), angle(new Cardan()), Elements(0), rotationalElements(0), l0(0.), E(0.), G(0.), A(0.), I1(0.), rho(0.), R1(0.), cEps0D(0.), cEps1D(0.), initialised(false), bound_ang_start(3, INIT, 0.), bound_ang_end(3, INIT, 0.), bound_ang_vel_start(3, INIT, 0.), bound_ang_vel_end(3, INIT, 0.) {
  }

  void FlexibleBody1sCosserat::updateh(double t, int k) {
    /* translational elements */
    FlexibleBody1s::updateh(t,k);

    /* rotational elements */
    for (int i = 0; i < (int) rotationDiscretization.size(); i++)
      rotationDiscretization[i]->computeh(getqRotationElement(t,i), getuRotationElement(t,i)); // compute attributes of finite element
    for (int i = 0; i < (int) rotationDiscretization.size(); i++)
      GlobalVectorContributionRotation(i, rotationDiscretization[i]->geth(), h[0]); // assemble
  }

  Contour1sNeutralCosserat* FlexibleBody1sCosserat::createNeutralPhase(const std::string & contourName) {
    VecInt transNodes(Elements);
    VecInt rotNodes(Elements);
    for (int i = 0; i < Elements; i++) {
      transNodes(i) = i;
      rotNodes(i) = i;
    }
    double uMin = 0;  // uMin has to be 0, otherwise Nurbscurve:globalInterpClosed():inv() fails;
    double uMax = 1;
    double nodeOffset = 0.5 * (uMax - uMin) / Elements;
    ncc = new Contour1sNeutralCosserat(contourName);
    ncc->setTransNodes(transNodes);
    ncc->setRotNodes(rotNodes);
    ncc->setNodeOffest(nodeOffset);
    ncc->setOpenStructure(openStructure);
    ncc->setFrameOfReference(getFrameOfReference());
    ncc->setAlphaStart(uMin);
    ncc->setAlphaEnd(uMax);

    addContour(ncc);

    return ncc;
return NULL;
  }
}

/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#include <config.h>

#include "contour_1s_neutral_factory.h"

#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/utils/rotarymatrices.h"

using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Contour1sNeutralFactory::Contour1sNeutralFactory(const std::string &name) :
      MBSim::Contour1s(name), uMin(0.), uMax(1.), degU(3), openStructure(false)
  {
  }

  Contour1sNeutralFactory::~Contour1sNeutralFactory() {
  }

  ContourFrame* Contour1sNeutralFactory::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

} /* namespace MBSimFlexibleBody */

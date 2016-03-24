/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#include <config.h>
#include "contour_2s_neutral_factory.h"
#include "mbsim/frames/floating_contour_frame.h"

using namespace std;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ContourFrame* Contour2sNeutralFactory::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

} /* namespace MBSimFlexibleBody */

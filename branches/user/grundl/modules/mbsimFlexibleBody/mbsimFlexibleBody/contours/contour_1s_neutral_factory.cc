/*
 * contour_1s_neutral_factory.h
 *
 *  Created on: 25.10.2013
 *      Author: zwang
 */

#include <config.h>

#include "contour_1s_neutral_factory.h"

#include <mbsim/utils/rotarymatrices.h>

namespace MBSimFlexibleBody {

  Contour1sNeutralFactory::Contour1sNeutralFactory(const std::string &name) :
      MBSim::Contour1s(name), uMin(0.), uMax(1.), degU(3), openStructure(false)
//#ifdef HAVE_OPENMBVCPPINTERFACE
//          , openMBVSpineExtrusion(0)
//#endif
  {
  }

  Contour1sNeutralFactory::~Contour1sNeutralFactory() {
    /* TODO: delete of spine-extrusion not possible!
     if(openMBVSpineExtrusion)
     delete openMBVSpineExtrusion;*/
  }




} /* namespace MBSimFlexibleBody */

/* Copyright (C) 2004-2009 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: markus.ms.schneider@gmail.com
 */

#include <config.h>
#include "mbsim/functions/kinetics/influence_function.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/contour.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void InfluenceFunction::initializeUsingXML(DOMElement *element) {
    Function<double(std::pair<Contour*, ContourFrame*>,std::pair<Contour*, ContourFrame*>)>::initializeUsingXML(element);
  }

  fmatvec::Vec2 InfluenceFunction::getZeta(double t, const std::pair<Contour*, ContourFrame*>& contourInfo) {
    return contourInfo.first->getZeta(t, contourInfo.second->evalPosition());
  }

}

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
 * Contact: foerg@users.berlios.de
 */

#include<config.h>
#include "mbsim/contours/line_segment.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(LineSegment, MBSIM%"LineSegment")

  LineSegment::LineSegment(const std::string& name, double l, double t, Frame *R) : RigidContour(name,R), length(l), thickness(t) {
  }

  LineSegment::LineSegment(const std::string& name, double l, Frame *R) : RigidContour(name,R), length(l), thickness(0.01) {
  }

  LineSegment::LineSegment(const std::string& name, Frame *R) : RigidContour(name,R), length(1), thickness(0.01) {
  }

  void LineSegment::init(InitStage stage) {
    if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          if(openMBVRigidBody) static_pointer_cast<OpenMBV::Cuboid>(openMBVRigidBody)->setLength(0,length,0);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  void LineSegment::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"length");
    setLength(getDouble(e));
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVLine ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  DOMElement* LineSegment::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
    addElementText(ele0,MBSIM%"length",length);
    return ele0;
  }

}


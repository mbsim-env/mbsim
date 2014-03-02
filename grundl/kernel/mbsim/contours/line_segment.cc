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

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, LineSegment, MBSIMNS"LineSegment")

  LineSegment::LineSegment(const std::string& name, double l, double t, Frame *R) : RigidContour(name,R), length(l), thickness(t) {
  }

  LineSegment::LineSegment(const std::string& name, double l, Frame *R) : RigidContour(name,R), length(l), thickness(0.01) {
  }

  LineSegment::LineSegment(const std::string& name, Frame *R) : RigidContour(name,R), length(1), thickness(0.01) {
  }

  void LineSegment::init(InitStage stage) {
    if(stage==MBSim::plot) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          if(openMBVRigidBody) ((OpenMBV::Cuboid*)openMBVRigidBody)->setLength(0,length,0);
        }
#endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);
  }

  void LineSegment::initializeUsingXML(TiXmlElement *element) {
    RigidContour::initializeUsingXML(element);
    TiXmlElement *e=element->FirstChildElement(MBSIMNS"length");
    setLength(getDouble(e));
#ifdef HAVE_OPENMBVCPPINTERFACE
    e=element->FirstChildElement(MBSIMNS"enableOpenMBV");
    if(e) {
      OpenMBVLine ombv;
      openMBVRigidBody=ombv.createOpenMBV(e); 
    }
#endif
  }

  TiXmlElement* LineSegment::writeXMLFile(TiXmlNode *parent) {
    TiXmlElement *ele0 = Contour::writeXMLFile(parent);
    addElementText(ele0,MBSIMNS"length",length);
    return ele0;
  }

}


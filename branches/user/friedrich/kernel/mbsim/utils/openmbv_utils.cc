/* Copyright (C) 2004-2013 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#include <config.h>
#include "mbsim/element.h"
#include "mbsim/utils/openmbv_utils.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

#ifdef HAVE_OPENMBVCPPINTERFACE
  void enableOpenMBVArrow(OpenMBV::Arrow* &arrow, const fmatvec::Vec3& dC, double tp, double d, double hD, double hL, OpenMBV::Arrow::Type &t, OpenMBV::Arrow::ReferencePoint &rP, double sL) {
    arrow=new OpenMBV::Arrow;
    arrow->setDiffuseColor(dC(0),dC(1),dC(2));
    arrow->setTransparency(tp);
    arrow->setDiameter(d);
    arrow->setHeadDiameter(hD);
    arrow->setHeadLength(hL);
    arrow->setType(t);
    arrow->setReferencePoint(rP);
    arrow->setScaleLength(sL);
  }

  void readOpenMBVArrow(TiXmlElement* e, fmatvec::Vec3& dC, double &tp, double &d, double &hD, double &hL, OpenMBV::Arrow::Type &t, OpenMBV::Arrow::ReferencePoint &rP, double &sL) {
      TiXmlElement *ee;
      ee=e->FirstChildElement(MBSIMNS"diffuseColor");
      if(ee) dC = Element::getVec(ee, 3);
      ee=e->FirstChildElement(MBSIMNS"transparency");
      if(ee) tp = Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMNS"diameter");
      if(ee) d = Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMNS"headDiameter");
      if(ee) hD = Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMNS"headLength");
      if(ee) hL = Element::getDouble(ee);
      ee=e->FirstChildElement(MBSIMNS"type");
      if(ee) {
        string typeStr=string(ee->GetText()).substr(1,string(ee->GetText()).length()-2);
        if(typeStr=="line")                 t = OpenMBV::Arrow::line;
        else if(typeStr=="fromHead")        t = OpenMBV::Arrow::fromHead;
        else if(typeStr=="toHead")          t = OpenMBV::Arrow::toHead;
        else if(typeStr=="bothHeads")       t = OpenMBV::Arrow::bothHeads;
        else if(typeStr=="fromDoubleHead")  t = OpenMBV::Arrow::fromDoubleHead;
        else if(typeStr=="toDoubleHead")    t = OpenMBV::Arrow::toDoubleHead;
        else if(typeStr=="bothDoubleHeads") t = OpenMBV::Arrow::bothDoubleHeads;
      }
      ee=e->FirstChildElement(MBSIMNS"referencePoint");
      if(ee) {
        string rPs=string(ee->GetText()).substr(1,string(ee->GetText()).length()-2);
        if(rPs=="toPoint")   rP = OpenMBV::Arrow::toPoint;
        else if(rPs=="fromPoint") rP = OpenMBV::Arrow::fromPoint;
        else if(rPs=="midPoint")  rP = OpenMBV::Arrow::midPoint;
      }
      ee = e->FirstChildElement(MBSIMNS"scaleLength");
      if(ee) sL = Element::getDouble(ee);
  }
#endif

}

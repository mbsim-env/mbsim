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
#include "mbsim/utils/openmbv_arrow_utils.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;

namespace MBSim {

#ifdef HAVE_OPENMBVCPPINTERFACE
//  void enableOpenMBVArrow(OpenMBV::Arrow* &arrow, const Vec3& dC, double tp, double d, double hD, double hL, const OpenMBV::Arrow::Type &t, const OpenMBV::Arrow::ReferencePoint &rP, double sL) {
//  }

  OpenMBV::Arrow* enableOpenMBVArrow(const Vec3 &dC_, double tp_, const OpenMBV::Arrow::Type &t_, const OpenMBV::Arrow::ReferencePoint &rP_, double sL_, double sS_, TiXmlElement* e) {
    OpenMBV::Arrow *arrow=new OpenMBV::Arrow;
    double sL=sL_, sS=sS_, tp=tp_;
    OpenMBV::Arrow::Type t = t_;
    OpenMBV::Arrow::ReferencePoint rP = rP_;
    Vec3 dC=dC_;
    if(e) {
      TiXmlElement *ee;
      ee=e->FirstChildElement(MBSIMNS"diffuseColor");
      if(ee) dC = Element::getVec(ee, 3);
      ee=e->FirstChildElement(MBSIMNS"transparency");
      //    if(ee) tp = Element::getDouble(ee);
      //    ee = e->FirstChildElement(MBSIMNS"diameter");
      //    if(ee) d = Element::getDouble(ee);
      //    ee = e->FirstChildElement(MBSIMNS"headDiameter");
      //    if(ee) hD = Element::getDouble(ee);
      //    ee = e->FirstChildElement(MBSIMNS"headLength");
      //    if(ee) hL = Element::getDouble(ee);
      //    ee=e->FirstChildElement(MBSIMNS"type");
      //    if(ee) {
      //      string typeStr=string(ee->GetText()).substr(1,string(ee->GetText()).length()-2);
      //      if(typeStr=="line")                 t = OpenMBV::Arrow::line;
      //      else if(typeStr=="fromHead")        t = OpenMBV::Arrow::fromHead;
      //      else if(typeStr=="toHead")          t = OpenMBV::Arrow::toHead;
      //      else if(typeStr=="bothHeads")       t = OpenMBV::Arrow::bothHeads;
      //      else if(typeStr=="fromDoubleHead")  t = OpenMBV::Arrow::fromDoubleHead;
      //      else if(typeStr=="toDoubleHead")    t = OpenMBV::Arrow::toDoubleHead;
      //      else if(typeStr=="bothDoubleHeads") t = OpenMBV::Arrow::bothDoubleHeads;
      //    }
      //    ee=e->FirstChildElement(MBSIMNS"referencePoint");
      //    if(ee) {
      //      string rPs=string(ee->GetText()).substr(1,string(ee->GetText()).length()-2);
      //      if(rPs=="toPoint")   rP = OpenMBV::Arrow::toPoint;
      //      else if(rPs=="fromPoint") rP = OpenMBV::Arrow::fromPoint;
      //      else if(rPs=="midPoint")  rP = OpenMBV::Arrow::midPoint;
      //    }
      ee = e->FirstChildElement(MBSIMNS"scaleLength");
      if(ee) sL = Element::getDouble(ee);
      ee = e->FirstChildElement(MBSIMNS"scaleSize");
      if(ee) sS = Element::getDouble(ee);

      // pass a OPENMBV_ID processing instruction to the OpenMBV Frame object
      for(TiXmlNode *child=e->FirstChild(); child; child=child->NextSibling()) {
        TiXmlUnknown *unknown=child->ToUnknown();
        const size_t length=strlen("?OPENMBV_ID ");
        if(unknown && unknown->ValueStr().substr(0, length)=="?OPENMBV_ID ")
          arrow->setID(unknown->ValueStr().substr(length, unknown->ValueStr().length()-length-1));
      }
    }
    arrow->setDiffuseColor(dC(0),dC(1),dC(2));
    arrow->setTransparency(tp);
    arrow->setDiameter(0.25*sS);
    arrow->setHeadDiameter(0.5*sS);
    arrow->setHeadLength(0.75*sS);
    arrow->setType(t);
    arrow->setReferencePoint(rP);
    arrow->setScaleLength(sL);
    return arrow;
  }
#endif

}

/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsimFlexibleBody/utils/openmbv_utils.h"
#include "mbsimFlexibleBody/namespace.h"
#include "mbsim/element.h"

using namespace std;
using namespace MBXMLUtils;
using namespace fmatvec;
using namespace MBSim;
using namespace xercesc;

namespace MBSimFlexibleBody {

  OpenMBVFlexibleBody::OpenMBVFlexibleBody(unsigned int cR, double minCol, double maxCol, const fmatvec::Vec3 &dc, double tp, double ps, double lw) : OpenMBVDynamicColoredBody(cR,minCol,maxCol,dc,tp,ps,lw) {
    cRL.resize(12);
    cRL[0]="none";
    cRL[1]="xDisplacement";
    cRL[2]="yDisplacement";
    cRL[3]="zDisplacement";
    cRL[4]="totalDisplacement";
    cRL[5]="xxStress";
    cRL[6]="yyStress";
    cRL[7]="zzStress";
    cRL[8]="xyStress";
    cRL[9]="yzStress";
    cRL[10]="zxStress";
    cRL[11]="equivalentStress";
  }

  void OpenMBVExternalFlexibleFfrBody::initializeUsingXML(DOMElement *e) {
    OpenMBVFlexibleBody::initializeUsingXML(e);
    DOMElement *ee = E(e)->getFirstElementChildNamed(MBSIMFLEX%"visualization");
    if(ee) {
      string str=string(X()%E(ee)->getFirstTextChild()->getData()).substr(1,string(X()%E(ee)->getFirstTextChild()->getData()).length()-2);
      if(str=="points") visu=points;
      else if(str=="lines") visu=lines;
      else if(str=="faces") visu=faces;
    }
  }

  shared_ptr<OpenMBV::FlexibleBody> OpenMBVExternalFlexibleFfrBody::createOpenMBV() {
    shared_ptr<OpenMBV::FlexibleBody> object;
    if(visu==points)
      object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicPointSet>();
    else if(visu==lines)
      object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicIndexedLineSet>();
    else
      object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicIndexedFaceSet>();
    initializeObject(object);
    return object;
  }

  shared_ptr<OpenMBV::DynamicNurbsCurve> OpenMBVDynamicNurbsCurve::createOpenMBV() {
    shared_ptr<OpenMBV::DynamicNurbsCurve> object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicNurbsCurve>();
    initializeObject(object);
    return object;
  }

  shared_ptr<OpenMBV::DynamicNurbsSurface> OpenMBVDynamicNurbsSurface::createOpenMBV() {
    shared_ptr<OpenMBV::DynamicNurbsSurface> object = OpenMBV::ObjectFactory::create<OpenMBV::DynamicNurbsSurface>();
    initializeObject(object);
    return object;
  }

}

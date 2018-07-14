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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _MBSIMFLEX_OPENMBV_UTILS_H_
#define _MBSIMFLEX_OPENMBV_UTILS_H_

#include <mbsim/utils/openmbv_utils.h>
#include <openmbvcppinterface/dynamicnurbscurve.h>
#include <openmbvcppinterface/dynamicnurbssurface.h>

namespace MBSimFlexibleBody {

  class OpenMBVDynamicNurbsCurve : public MBSim::OpenMBVColoredBody {
    public:
      OpenMBVDynamicNurbsCurve(const fmatvec::Vec3 &dc="[-1;1;1]", double tp=0, double ps=0, double lw=0) : MBSim::OpenMBVColoredBody(dc,tp,ps,lw) { }
      std::shared_ptr<OpenMBV::DynamicNurbsCurve> createOpenMBV();
  };

  class OpenMBVDynamicNurbsSurface : public MBSim::OpenMBVColoredBody {
    public:
      OpenMBVDynamicNurbsSurface(const fmatvec::Vec3 &dc="[-1;1;1]", double tp=0, double ps=0, double lw=0) : MBSim::OpenMBVColoredBody(dc,tp,ps,lw) { }
      std::shared_ptr<OpenMBV::DynamicNurbsSurface> createOpenMBV();
  };

}

#endif

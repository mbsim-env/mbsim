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
 * Contact: martin.o.foerg@googlemail.com
 *          rzander@users.berlios.de
 */

#ifndef _OPENMBV_ARROW_UTILS_H_
#define _OPENMBV_ARROW_UTILS_H_

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <boost/parameter/name.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <openmbvcppinterface/arrow.h>
#include <fmatvec/fmatvec.h>
#endif

namespace MBSim {
#ifdef HAVE_OPENMBVCPPINTERFACE
  BOOST_PARAMETER_NAME(scaleLength)
  BOOST_PARAMETER_NAME(scaleSize)
  BOOST_PARAMETER_NAME(type)
  BOOST_PARAMETER_NAME(referencePoint)
  BOOST_PARAMETER_NAME(diffuseColor)
  BOOST_PARAMETER_NAME(transparency)
  //void enableOpenMBVArrow(OpenMBV::Arrow* &arrow, const fmatvec::Vec3& dC, double tp, double d, double hD, double hL, const OpenMBV::Arrow::Type &t, const OpenMBV::Arrow::ReferencePoint &rP, double sL);
  OpenMBV::Arrow* enableOpenMBVArrow(const fmatvec::Vec3 &dC_, double tp_, const OpenMBV::Arrow::Type &t_, const OpenMBV::Arrow::ReferencePoint &rP_, double sL_, double sS_, MBXMLUtils::TiXmlElement* e=0);
#endif
}

#endif

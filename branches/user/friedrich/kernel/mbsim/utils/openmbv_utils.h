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

#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <boost/parameter/name.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <openmbvcppinterface/arrow.h>
#include <fmatvec/fmatvec.h>
#endif

namespace MBSim {
#ifdef HAVE_OPENMBVCPPINTERFACE
  BOOST_PARAMETER_NAME(size)
  BOOST_PARAMETER_NAME(offset)
  BOOST_PARAMETER_NAME(scaleLength)
  BOOST_PARAMETER_NAME(type)
  BOOST_PARAMETER_NAME(referencePoint)
  BOOST_PARAMETER_NAME(diameter)
  BOOST_PARAMETER_NAME(headDiameter)
  BOOST_PARAMETER_NAME(headLength)
  BOOST_PARAMETER_NAME(diffuseColor)
  BOOST_PARAMETER_NAME(transparency)
  void enableOpenMBVArrow(OpenMBV::Arrow* &arrow, const fmatvec::Vec3& dC, double tp, double d, double hD, double hL, OpenMBV::Arrow::Type &t, OpenMBV::Arrow::ReferencePoint &rP, double sL);
  void readOpenMBVArrow(MBXMLUtils::TiXmlElement* e, fmatvec::Vec3& dC, double &tp, double &d, double &hD, double &hL, OpenMBV::Arrow::Type &t, OpenMBV::Arrow::ReferencePoint &rP, double &sL);
#endif
}

#endif

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
 */

#ifndef _BOOST_PARAMETERS_H_
#define _BOOST_PARAMETERS_H_

// set the maximal number or arguments (only boost libs use only 5 newer use already 8)
#define BOOST_PARAMETER_MAX_ARITY 8

#include <boost/parameter/name.hpp>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/preprocessor.hpp>

namespace MBSim {

  BOOST_PARAMETER_NAME(scaleLength)
  BOOST_PARAMETER_NAME(scaleSize)
  BOOST_PARAMETER_NAME(type)
  BOOST_PARAMETER_NAME(referencePoint)
  BOOST_PARAMETER_NAME(diffuseColor)
  BOOST_PARAMETER_NAME(transparency)
  BOOST_PARAMETER_NAME(size)
  BOOST_PARAMETER_NAME(offset)
  BOOST_PARAMETER_NAME(length)
  BOOST_PARAMETER_NAME(xLength)
  BOOST_PARAMETER_NAME(yLength)
  BOOST_PARAMETER_NAME(zLength)
  BOOST_PARAMETER_NAME(polynomialPoints)
  BOOST_PARAMETER_NAME(circularPoints)
  BOOST_PARAMETER_NAME(numberOfCoils)
  BOOST_PARAMETER_NAME(nominalLength)
  BOOST_PARAMETER_NAME(springRadius)
  BOOST_PARAMETER_NAME(scaleFactor)
  BOOST_PARAMETER_NAME(crossSectionRadius)
  BOOST_PARAMETER_NAME(nodes)
  BOOST_PARAMETER_NAME(etaNodes)
  BOOST_PARAMETER_NAME(xiNodes)

}

#endif


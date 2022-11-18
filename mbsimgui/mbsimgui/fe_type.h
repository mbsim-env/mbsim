/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2022 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _FE_TYPE_H_
#define _FE_TYPE_H_

#include <fmatvec/fmatvec.h>

namespace MBSimGUI {

  class FiniteElementType  {
    public:
      virtual ~FiniteElementType() = default;
      virtual double N(int i, double x, double y, double z) const = 0;
      virtual double dNdq(int i, int j, double x, double y, double z) const = 0;
      virtual int getNumberOfNodes() const = 0;
      virtual int getNumberOfIntegrationPoints() const = 0;
      virtual int getNumberOfFaces() const = 0;
      virtual int getNumberOfIndicesPerFace() const = 0;
      virtual const fmatvec::Vec3& getNaturalCoordinates(int i) const = 0;
      virtual const fmatvec::Vec3& getIntegrationPoint(int i) const = 0;
      virtual double getWeight(int i) const = 0;
      virtual int getOmbvIndex(int i, int j) const = 0;
  };

}

#endif

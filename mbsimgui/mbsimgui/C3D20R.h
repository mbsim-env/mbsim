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

#ifndef _C3D20R_H_
#define _C3D20R_H_

#include "C3D20Base.h"

namespace MBSimGUI {

  class C3D20R : public C3D20Base {
    public:
      C3D20R();

      int getNumberOfIntegrationPoints() const override { return 8; }
      const fmatvec::Vec3& getIntegrationPoint(int i) const override { return rI[i]; }
      double getWeight(int i) const override { return wI[i]; }

    private:
      fmatvec::Vec3 rI[8];
      double wI[8];
  };

}

#endif

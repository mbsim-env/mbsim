/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef TAYLOR_H_
#define TAYLOR_H_

#include <mbsim/functions/function.h>

namespace MBSimFlexibleBody {

  template<typename T0, typename T1=T0, typename T2=T1> class Taylor {
    public:
      Taylor() { }
      Taylor(const T0 &M0_) : M0(M0_) { }
      Taylor(const T0 &M0_, const T1 &M1_) : M0(M0_), M1(M1_) { }
      Taylor(const T0 &M0_, const T1 &M1_, const T2 &M2_) : M0(M0_), M1(M1_), M2(M2_) { }
      void setM0(const T0 &M0_) { M0 = M0_; }
      void setM1(const T1 &M1_) { M1 = M1_; }
      void setM2(const T2 &M2_) { M2 = M2_; }
      const T0& getM0() const { return M0; }
      const T1& getM1() const { return M1; }
      const T2& getM2() const { return M2; }
      T0& getM0() { return M0; }
      T1& getM1() { return M1; }
      T2& getM2() { return M2; }
    private:
      T0 M0;
      T1 M1;
      T2 M2;
  };

}

#endif /*TAYLOR_H_*/


/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef _ROTATION_ABOUT_THREE_AXES_H_
#define _ROTATION_ABOUT_THREE_AXES_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /*!
   * \brief rotation class for rotation about three axes
   */
  template<class Arg> 
  class RotationAboutThreeAxes : public Function<fmatvec::RotMat3(Arg)> {
    protected:
      fmatvec::RotMat3 A;
      fmatvec::Mat3xV J, Jd;
    public:
      RotationAboutThreeAxes() : J(3), Jd(3) { }
      int getArgSize() const override { return 3; }
      virtual Function<fmatvec::MatV(Arg)>* getMappingFunction() const { return nullptr; }
      virtual Function<fmatvec::MatV(Arg)>* getTransformedMappingFunction() const { return nullptr; }
  };

}

#endif

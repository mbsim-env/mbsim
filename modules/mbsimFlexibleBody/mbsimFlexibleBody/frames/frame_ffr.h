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

#ifndef _FRAME_FFR_H__
#define _FRAME_FFR_H__

#include "mbsim/frame.h"

namespace MBSimFlexibleBody {

  class FrameFFR : public MBSim::Frame {
    public:
      /**
       * \brief constructor
       * \param name of coordinate system
       */
      FrameFFR(const std::string &name = "dummy") : MBSim::Frame(name) { }

      std::string getType() const { return "FrameFFR"; }

      /* INTERFACE FOR DERIVED CLASSES */
      /***************************************************/

      fmatvec::MatV& getJacobianOfDeformation(int j=0,  bool check=true) { assert((not check) or (not updateJac[j])); return WJD[j]; }
      void setJacobianOfDeformation(const fmatvec::MatV &J, int j=0) { WJD[j] = J; }
      const fmatvec::MatV& getJacobianOfDeformation(double t, int j=0) { if(updateJac[j]) updateJacobians(t,j); return WJD[j]; }

    protected:

      fmatvec::MatV WJD[3];
  };

}

#endif


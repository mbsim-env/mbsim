/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _FLOATING_FRAME_H__
#define _FLOATING_FRAME_H__

#include "mbsim/frame.h"

namespace MBSim {
  class Contour;
}

namespace MBSimFlexibleBody {

  /**
   * \brief cartesian frame on rigid bodies 
   * \author Martin Foerg
   */
  class FloatingFrame : public MBSim::Frame {

    public:
      FloatingFrame(const std::string &name = "dummy", MBSim::Contour* contour_ = NULL) : Frame(name), contour(contour_) { }

      std::string getType() const { return "FloatingFrame"; }

      virtual void init(InitStage stage);

      void updatePositions(double t);
      void updateVelocities(double t); 
      void updateAccelerations(double t); 
      void updateJacobians(double t, int j=0);
      void updateGyroscopicAccelerations(double t);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      MBSim::Contour *contour;
      fmatvec::Vec2 zeta;
  };

}

#endif


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

#ifndef _FIXED_RELATIVE_FRAME_H__
#define _FIXED_RELATIVE_FRAME_H__

#include "mbsim/frames/frame.h"

namespace MBSim {

  /**
   * \brief cartesian frame on rigid bodies 
   * \author Martin Foerg
   */
  class FixedRelativeFrame : public Frame {

    public:
      FixedRelativeFrame(const std::string &name = "dummy", const fmatvec::Vec3 &r=fmatvec::Vec3(), const fmatvec::SqrMat3 &A=fmatvec::SqrMat3(fmatvec::EYE), Frame *refFrame=0) : Frame(name), R(refFrame), RrRP(r), ARP(A) {
      }

      std::string getType() const { return "FixedRelativeFrame"; }

      virtual void init(InitStage stage);

      void setRelativePosition(const fmatvec::Vec3 &r) { RrRP = r; }
      void setRelativeOrientation(const fmatvec::SqrMat3 &A) { ARP = A; }
      void setFrameOfReference(Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARP; }
      const Frame* getFrameOfReference() const { return R; }

      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(0.); return WrRP; }

      void updatePositions(double t);
      void updateVelocities(double t); 
      void updateAccelerations(double t); 
      void updateJacobians(double t, int j=0);
      void updateGyroscopicAccelerations(double t);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      Frame *R;
      fmatvec::Vec3 RrRP, WrRP;
      fmatvec::SqrMat3 ARP;
      std::string saved_frameOfReference;
  };

}

#endif


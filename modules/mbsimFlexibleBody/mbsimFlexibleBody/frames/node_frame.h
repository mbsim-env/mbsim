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

#ifndef _NODE_FRAME_H__
#define _NODE_FRAME_H__

#include "mbsim/frames/frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief cartesian frame on nodes of flexible bodies
   * \author Kilian Grundl
   */
  class NodeFrame : public MBSim::Frame {

    public:
      NodeFrame(const std::string &name = "dummy", int node_ = 0) : Frame(name), node(node_), updAngles(true), updDerAngles(true) { }

      std::string getType() const { return "NodeFrame"; }

      void setNodeNumber(int node_) { node = node_; }
      int getNodeNumber() const { return node; }

      void updatePositions(double t);
      void updateVelocities(double t);
      void updateAccelerations(double t);
      void updateJacobians(double t, int j=0);
      void updateGyroscopicAccelerations(double t);
      void updateAngles(double t);
      void updateDerAngles(double t);

      const fmatvec::Vec3& getAngles(bool check=true) const { assert((not check) or (not updAngles)); return angles; }
      const fmatvec::Vec3& getDerivativeOfAngles(bool check=true) const { assert((not check) or (not updDerAngles)); return derAngles; }
      void setAngles(const fmatvec::Vec3 &angles_) { angles = angles_; }
      void setDerivativeOfAngles(const fmatvec::Vec3 &derAngles_ ) { derAngles = derAngles_; }

      void resetUpToDate();

      const fmatvec::Vec3& getAngles(double t) { if(updAngles) updateAngles(t); return angles; }
      const fmatvec::Vec3& getDerivativeOfAngles(double t) { if(updDerAngles) updateDerAngles(t); return derAngles; }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      /*!
       * \brief node number of the frame
       */
      int node;

      fmatvec::Vec3 angles, derAngles;

      bool updAngles, updDerAngles;
  };

}

#endif

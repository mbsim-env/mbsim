/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _DISTRIBUTING_FFR_INTERFACE_NODE_FRAME_H__
#define _DISTRIBUTING_FFR_INTERFACE_NODE_FRAME_H__

#include "mbsimFlexibleBody/frames/generic_ffr_interface_node_frame.h"

namespace MBSimFlexibleBody {

  /**
   * \brief distributing interface node frame for flexible ffr bodies
   * \author Martin Förg
   */
  class DistributingFfrInterfaceNodeFrame : public GenericFfrInterfaceNodeFrame {

    public:
      DistributingFfrInterfaceNodeFrame(const std::string &name = "dummy") : GenericFfrInterfaceNodeFrame(name) { }

      void setElementNumbers(const fmatvec::VecVI &elements_) { elements <<= elements_; }
      const fmatvec::VecVI& getElementNumbers() const { return elements; }

      void setFaceNumber(int faceNumber_) { faceNumber = faceNumber_; }
      int getFaceNumber() const { return faceNumber; }

      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      fmatvec::VecVI elements;
      int faceNumber;
  };

}

#endif

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

#ifndef _CONTOUR_FRAME_H__
#define _CONTOUR_FRAME_H__

#include "mbsim/frames/frame.h"

namespace MBSim {

  class ContourFrame : public Frame {

    public:
      ContourFrame(const std::string &name = "dummy", const fmatvec::Vec2 &zeta_ = fmatvec::Vec2()) : Frame(name), zeta(zeta_) { }

      std::string getType() const { return "ContourFrame"; }

      const fmatvec::Vec2& getZeta() const { return zeta; }
      void setZeta(const fmatvec::Vec2 &zeta_) { zeta = zeta_; }

      double getEta() const { return zeta(0); }
      double getXi() const { return zeta(1); }
      void setEta(double eta) { zeta(0) = eta; }
      void setXi(double xi) { zeta(1) = xi; }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      /*!
       * \brief contour parameters of the frame
       */
      fmatvec::Vec2 zeta;
  };

}

#endif

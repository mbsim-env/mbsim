/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _CONTOUR1S_H_
#define _CONTOUR1S_H_

#include "mbsim/contours/contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSimFlexibleBody {

  /** 
   * \brief basic class for contours described by one contour parameter \f$s\f$
   * \author Roland Zander
   * \date 2009-04-20 frame-concept (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour1s : public MBSim::Contour {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      Contour1s(const std::string &name) : Contour(name) { }

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Contour1s"; }
      void init(InitStage stage_);
      void plot(double t, double dt);
      /***************************************************/

      virtual fmatvec::Vec3 getKt(const fmatvec::Vec2 &zeta);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(const boost::shared_ptr<OpenMBV::SpineExtrusion> &spineExtrusion) { openMBVSpineExtrusion = spineExtrusion; }
      boost::shared_ptr<OpenMBV::SpineExtrusion>& getOpenMBVSpineExtrusion() { return openMBVSpineExtrusion; }
#endif

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      /*!
       * \brief body for the spine extrusion for visualisation of the 1s-body
       */
      boost::shared_ptr<OpenMBV::SpineExtrusion> openMBVSpineExtrusion;

#endif

  };

}

#endif /* _CONTOUR1S_H_ */

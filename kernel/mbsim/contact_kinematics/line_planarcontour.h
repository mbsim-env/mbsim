/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _CONTACT_KINEMATICS_LINE_PLANAR_CONTOUR_H_
#define _CONTACT_KINEMATICS_LINE_PLANAR_CONTOUR_H_

#include "contact_kinematics.h"

namespace MBSim {

  class Line;
  class FuncPairPlanarContourLine;

  /**
   * \brief pairing line to planar contour
   * \author Martin Foerg
   */
  class ContactKinematicsLinePlanarContour : public ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsLinePlanarContour() = default;

      /**
       * \brief destructor
       */
      ~ContactKinematicsLinePlanarContour() override;
      
      /* INHERITED INTERFACE */
      void assignContours(const std::vector<Contour*> &contour) override;
      void updateg(SingleContact &contact, int i=0) override;
      void updatewb(SingleContact &contact, int i=0) override;
      /***************************************************/

      void setSearchAllContactPoints(bool searchAllCP_=true) override { searchAllCP = searchAllCP_; }
      void setInitialGuess(const fmatvec::MatV &zeta0_) override;

    private:
      /**
       * \brief contour index
       */
      int iline{0};
      int iplanarcontour{0};

      /**
       * \brief contour classes
       */
      Line *line{nullptr};
      Contour *planarcontour{nullptr};

      /**
       * \brief root function
       */
      FuncPairPlanarContourLine *func;

      bool searchAllCP{false};

      std::vector<double> zeta0;
  };

}

#endif

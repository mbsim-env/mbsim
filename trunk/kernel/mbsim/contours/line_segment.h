/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "mbsim/contour.h"

namespace MBSim {

  class ContourPointData;

  /**
   * \brief line segment with two bounds
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */ 
  class LineSegment : public MBSim::RigidContour {	
    public:
      /**
       * \brief constructor
       * \param name of line
       */
      LineSegment(const std::string& name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "LineSegment"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual double computeCurvature(ContourPointData &cp) { return 0; } 
#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true, double size=1., int number=10);
#endif

      void setBounds(const fmatvec::Vec &bound_) {bound = bound_;}
      const fmatvec::Vec& getBounds() const { return bound; }
      /***************************************************/
    private:
      fmatvec::Vec bound;
  };      
}

#endif /* _LINE_H_ */


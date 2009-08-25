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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _COMPOUND_CONTOUR_H_
#define _COMPOUND_CONTOUR_H_

#include "mbsim/contour.h"
#include "mbsim/contours/contour_interpolation.h"

namespace MBSim {

  /**
   * \brief contour consisting of primitive contour elements
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler) 
   */
  class CompoundContour : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CompoundContour(const std::string &name);

      /* INHERITED INTERFACE OF CONTOUR */
      void setReferencePosition(const fmatvec::Vec &WrOP);
      void setReferenceOrientation(const fmatvec::SqrMat &AWC);
      void setReferenceVelocity(const fmatvec::Vec &WvP);
      void setReferenceAngularVelocity(const fmatvec::Vec &WomegaC);
      void setReferenceJacobianOfTranslation(const fmatvec::Mat &WJP);
      void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec &WjP);
      void setReferenceJacobianOfRotation(const fmatvec::Mat &WJR);
      void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec &WjR);
      /***************************************************/

      void init(InitStage stage);
      Contour* getContourElement(int i) { return element[i]; }
      void addContourElement(Contour* ce, const fmatvec::Vec& re);
      unsigned int getNumberOfElements() { return element.size(); }

    private:
      std::vector<Contour*> element;
      std::vector<fmatvec::Vec> Kr, Wr;
  };
}

#endif /* _COMPOUND_CONTOUR_H_ */


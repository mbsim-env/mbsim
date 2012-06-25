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
  class CompoundContour : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      CompoundContour(const std::string &name);

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "CompoundContour"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void setReferencePosition(const fmatvec::FVec &WrOP);
      void setReferenceOrientation(const fmatvec::FSqrMat &AWC);
      void setReferenceVelocity(const fmatvec::FVec &WvP);
      void setReferenceAngularVelocity(const fmatvec::FVec &WomegaC);
      void setReferenceJacobianOfTranslation(const fmatvec::FVMat &WJP);
      void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::FVec &WjP);
      void setReferenceJacobianOfRotation(const fmatvec::FVMat &WJR);
      void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::FVec &WjR);
      /***************************************************/

      void init(InitStage stage);
      Contour* getContourElement(int i) { return element[i]; }
      void addContourElement(Contour* ce, const fmatvec::FVec& re);
      unsigned int getNumberOfElements() { return element.size(); }

      void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      void updateJacobiansForFrame(ContourPointData &cp);

    private:
      std::vector<Contour*> element;
      std::vector<fmatvec::FVec> Kr, Wr;
  };
}

#endif /* _COMPOUND_CONTOUR_H_ */


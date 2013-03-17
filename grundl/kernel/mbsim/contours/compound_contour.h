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
      std::string getType() const {
        return "CompoundContour";
      }
      virtual void plot(double t, double dt);
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Group* getOpenMBVGrp() { return openMBVGroup; }
#endif
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
//      void setReferencePosition(const fmatvec::Vec3 &WrOP);
//      void setReferenceOrientation(const fmatvec::SqrMat3 &AWC);
//      void setReferenceVelocity(const fmatvec::Vec3 &WvP);
//      void setReferenceAngularVelocity(const fmatvec::Vec3 &WomegaC);
//      void setReferenceJacobianOfTranslation(const fmatvec::Mat3V &WJP);
//      void setReferenceGyroscopicAccelerationOfTranslation(const fmatvec::Vec3 &WjP);
//      void setReferenceJacobianOfRotation(const fmatvec::Mat3V &WJR);
//      void setReferenceGyroscopicAccelerationOfRotation(const fmatvec::Vec3 &WjR);
      /***************************************************/

      void init(InitStage stage);
      Contour* getContourElement(int i) {
        return element[i];
      }
      Contour* getContour(int i) { return element[i]; }
      void addContour(RigidContour* ce);
      void addFrame(FixedRelativeFrame* f);
      unsigned int getNumberOfElements() { return element.size(); }

      void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      void updateJacobiansForFrame(ContourPointData &cp);

      virtual void updateStateDependentVariables(double t);
      virtual void updateJacobians(double t, int j=0);
      virtual void updateStateDerivativeDependentVariables(const fmatvec::Vec &ud, double t);

    protected:
      /*!
       * \brief list of all subelements
       */
      std::vector<RigidContour*> element;

      /*!
       * \brief List of all frames on the contour
       */
      std::vector<FixedRelativeFrame*> frame;

      /*!
       * \brief Orientations of the single elements in the contour frame
       */
      std::vector<fmatvec::SqrMat3> AIK;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* openMBVGroup;
#endif
  };
}

#endif /* _COMPOUND_CONTOUR_H_ */


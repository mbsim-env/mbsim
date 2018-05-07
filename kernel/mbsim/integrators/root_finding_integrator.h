/* Copyright (C) 2004-2018  Martin Förg
 
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
 * Contact:
 *   martin.o.foerg@googlemail.com
 *
 */

#ifndef _ROOT_FINDING_INTEGRATOR_H_
#define _ROOT_FINDING_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** \brief Integrator with root-finding
   */
  class RootFindingIntegrator : public Integrator {

    protected:

      // Helper function to check if svLast and svStepEnd has a sign change in any element.
      bool signChangedWRTsvLast(const fmatvec::Vec &svStepEnd) const;

      /** root-finding accuracy */
      double dtRoot{1e-10};

      /** plot on root */
      bool plotOnRoot{false};

       /** tolerance for position constraints */
      double gMax{-1};
      /** tolerance for velocity constraints */
      double gdMax{-1};

      fmatvec::Vec svLast;
      bool shift{false};

    public:

      //! Define the root-finding accuracy
      void setRootFindingAccuracy(double dtRoot_) { dtRoot = dtRoot_; }

      //! Define wether to trigger a plot before and after each found root.
      void setPlotOnRoot(bool b) { plotOnRoot = b; }

      //! Set the maximum allowed position drift.
      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }

      //! Set the maximum allowed velocity drift.
      void setToleranceForVelocityConstraints(double gdMax_) { gdMax = gdMax_; }

      //! Get the maximum allowed position drift.
      double getToleranceForPositionConstraints() { return gMax; }

      //! Get the maximum allowed velocity drift.
      double getToleranceForVelocityConstraints() { return gdMax; }

      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif

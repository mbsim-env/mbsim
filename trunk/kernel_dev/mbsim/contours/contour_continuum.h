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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _CONTOURCONTINUUM_H_
#define _CONTOURCONTINUUM_H_

#include "mbsim/contour.h"

namespace MBSim {

  /** 
   * \brief basic class for contours described by a parametrisation
   * \author Thorsten Schindler
   * \date 2009-04-20 initial commit (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   * \date 2009-08-16 fix in template usage (Thorsten Schindler)
   */
  template <class AT>
    class ContourContinuum : public Contour {
      public:
        /**
         * \brief constructor 
         * \param name of contour
         */
        ContourContinuum(const std::string &name) : Contour(name) {}

        /* INHERITED INTERFACE OF ELEMENT */
        virtual std::string getType() const { return "ContourContinuum"; }
        /***************************************************/

        /* INTERFACE FOR DERIVED CLASSES */
        /**
         * \brief compute necessary parameters for contact kinematics root function
         * \param contour point data
         */
        virtual void computeRootFunctionPosition(ContourPointData &cp) = 0;
        virtual void computeRootFunctionFirstTangent(ContourPointData &cp) = 0;
        virtual void computeRootFunctionNormal(ContourPointData &cp) = 0;
        virtual void computeRootFunctionSecondTangent(ContourPointData &cp) = 0;

        /* GETTER / SETTER */
        void setAlphaStart(AT as_) { as = as_; }
        void setAlphaEnd(AT ae_) { ae = ae_; }
        const AT& getAlphaStart() const { return as; }
        const AT& getAlphaEnd() const { return ae; }
        void setNodes(const std::vector<AT> &nodes_) { nodes = nodes_; }
        const std::vector<AT>& getNodes() const { return nodes; }
        /***************************************************/

      protected:
        AT as, ae;
        std::vector<AT> nodes;
    };

}

#endif /* _CONTOURCONTINUUM_H_ */


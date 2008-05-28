/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _IMPACT_RIGID_H_
#define _IMPACT_RIGID_H_

#include "contact_rigid.h"

namespace MBSim {

  /*! \brief Class for impacts */
  class ImpactRigid: public ContactRigid {

    protected:
	  /** normal restitution coefficient, limit velocity for additional term in relative velocity */
      double epsilonN, gd_grenz;

    public:
      /*! Constructor */
      ImpactRigid(const string &name);
      /*! Destructor */
      virtual ~ImpactRigid() {}
      /*! Function for single step */
      void projectGS(double dt);
      /*! Function for Gauss-Seidel (splitting) */
      void solveGS(double dt);
      /*! Test, if constraint iteration has converged */
      void checkForTermination(double dt);
      /*! Function for rootFinding with numerical Jacobian */
      void residualProj(double dt);
	  /*! Set normal restitution coefficient */
      void setNormalRestitutionCoefficient(double e) {epsilonN = e;}
  };

}

#endif /* _IMPACT_RIGID_H_ */

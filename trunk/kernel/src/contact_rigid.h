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

#ifndef _CONTACT_RIGID_H_
#define _CONTACT_RIGID_H_

#include "contact.h"

namespace MBSim {

  /*! \brief Class for rigid contacts */
  class ContactRigid : public Contact {
    protected:
      /** contact forces and moments for the two partners */
      Mat fF[2], fM[2];
      /** normal argument of prox */
      double argN;
      /** tangential argument of prox */
      Vec argT;
      /** current friction coefficient for Newton */
      double mue;
	  /*! Tests, if a contact is closed (=active) or not */
      void checkActive();
      /*! Updates friction coefficient with norm of relative tangential velocity */
      void updateFrictionCoefficient(double vel);

    public: 
	  /*! Constructor */
      ContactRigid(const string &name);
      /*! Clone constructor with new name, same parameters */
      ContactRigid(const ContactRigid *master, const string &name_) : Contact(master,name_) {}
	  /*! Destructor */
      virtual ~ContactRigid() {}
	  /*! Initialise contact description */
      void init();

      /*! Function for total step */
      void projectJ(double dt);
      /*! Function for single step */
      void projectGS(double dt);
      /*! Function for Gauss-Seidel (splitting) */
      void solveGS(double dt);
      /*! Update rFactors */
      void updaterFactors();
      /*! Function for rootFinding with numerical Jacobian */
      void residualProj(double dt);
      /*! Function for rootFinding with analytical Jacobian */
      void residualProjJac(double dt);
      /*! Test, if constraint iteration has converged */
      void checkForTermination(double dt);
      /*! Return information about constraint iteration */
      std::string getTerminationInfo(double dt);
	  /*! Set the contact directions */
      void updateKinetics(double t);
  };

}

#endif /* _CONTACT_RIGID_H_ */

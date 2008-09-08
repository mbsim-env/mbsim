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

#ifndef _RIGID_CONTACT_H_
#define _RIGID_CONTACT_H_

#include "contact.h"

namespace MBSim {

  /*! \brief Class for rigid contacts
   *
   * */
  class RigidContact : public Contact {

    protected:

      Mat fF[2], fM[2];
      double argN;
      Vec argT;
      double epsilonN, gd_grenz;

      void checkActive();

    public: 

      RigidContact(const string &name);

      /*! clone constructor with new name ! same parameters ! */
      RigidContact(const RigidContact *master, const string &name_) : Contact(master,name_) {}

      void init();

      /*! for time integration with projection methods for constraint and contact treatment */
      void projectJ(double dt);
      /*! for time integration with projection methods for constraint and contact treatment */
      void projectGS(double dt);
      /*! for time integration with projection methods for constraint and contact treatment */
      void solveGS(double dt);
      /*! for time integration with projection methods for constraint and contact treatment */
      void updaterFactors();

      /*! for time integration with projection methods for constraint and contact treatment */
      void residualProj(double dt);
      void residualProjJac(double dt);
      void checkForTermination(double dt);
      std::string getTerminationInfo(double dt);

      void updateKinetics(double t);
      void updateW(double t);

      void setNormalRestitutionCoefficient(double e) {epsilonN = e;}

      string getType() const {return "RigidContact";}

  };

  typedef RigidContact ContactRigid;

}

#endif   


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
#ifndef _CONTACT_FLEXIBLE_H_
#define _CONTACT_FLEXIBLE_H_

#include "contact.h"

namespace MBSim {

  /*! \brief Class for flexible contacts with regularised Stribeck friction
   * 
   * contact laws are based on gap distances and absolute relative velocities
   * with setStiffness and setDamping the flexible contact law is linear (DEPRECATED)
   */
  class ContactFlexible : public Contact {
    public:
      /*! Constructor */
      ContactFlexible(const string &name);
      /*! Clone Constructor  with new name */
      ContactFlexible(const ContactFlexible *master,const string &name_);
      /*! Destructor */
      virtual ~ContactFlexible();
      /*! Initialises the contact */
      void init();
      /*! Evaluates the fully functional force law */
      virtual void updateKinetics(double t);
      /*! Set stiffness with appropriate potential */
      void setStiffness(double c_);
      /*! Set stiffness function with correct sign according to potential and signed gap (acting against) */
      void setStiffnessFunction(DataInterfaceBase *c_fun_, bool DeleteDIB_c_=true);
      /*! Set potential according to signed gap */
      void setPotential(DataInterfaceBase *V_fun_, bool DeleteDIB_V_=true);
      /*! Set damping */
      void setDamping(double d_);
      /*! Set damping function with correct sign according to signed relative velocity (acting against) */
      void setDampingFunction(DataInterfaceBase *d_fun_, bool DeleteDIB_d_=true);
      /*! Set the marginal velocity for the regularised Stribeck friction */
      void setMarginalVelocity(double v) {gdT_grenz = v;}

      /*! Compute potential energy */
      double computePotentialEnergy();

    protected:
      /** contact forces and torques for both partners */
      Vec WF[2], WM[2];
      /** potential, stiffness and damping functions */
      DataInterfaceBase *V_fun, *c_fun, *d_fun;
      /** flag for deleting datainterfacebases */
      bool DeleteDIB_c, DeleteDIB_d, DeleteDIB_V;
      /** marginal velocity for regularisation */
      double gdT_grenz;
      /** flag for using stiffness function */
      bool flag_c;
  };

}

#endif /* _CONTACT_FLEXIBLE_H_ */

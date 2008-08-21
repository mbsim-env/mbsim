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
      void setStiffnessFunction(MBSim::DataInterfaceBase *c_fun_);
      /*! Set potential according to signed gap */
      void setPotential(MBSim::DataInterfaceBase *V_fun_);
      /*! Set damping */
      void setDamping(double d_);
      /*! Set damping function with correct sign according to signed relative velocity (acting against) */
      void setDampingFunction(MBSim::DataInterfaceBase *d_fun_);
	  /*! Set the marginal velocity for the regularised Stribeck friction */
      void setMarginalVelocity(double v);
      /*! Compute potential energy */
      double computePotentialEnergy();
      
    protected:
      /** contact forces and torques for both partners */
      Vec WF[2], WM[2];
	  /** potential, stiffness and damping functions */
      MBSim::DataInterfaceBase *V_fun, *c_fun, *d_fun;
      /** marginal velocity for regularisation */
      double gdT_grenz;
      /** flag for using stiffness function */
      bool flag_c;
  };
  
  inline void ContactFlexible::setStiffness(double c_) {if(warnLevel>0) cout << "WARNING (ContactFlexible::setStiffness): Deprecated function and only valid for linear contact law. Use setPotential and setStiffnessFunction instead." << endl; delete c_fun; c_fun = new FuncLinear(Vec(1,INIT,-c_),Vec(1,INIT,0.)); delete V_fun; V_fun = new FuncQuadratic(Vec(1,INIT,0.5*c_),Vec(1,INIT,0.),Vec(1,INIT,0.)); flag_c = true;}
  inline void ContactFlexible::setStiffnessFunction(MBSim::DataInterfaceBase *c_fun_) {delete c_fun; c_fun = c_fun_; flag_c = true;}
  inline void ContactFlexible::setPotential(MBSim::DataInterfaceBase *V_fun_) {delete V_fun; V_fun = V_fun_;}
  inline void ContactFlexible::setDamping(double d_) {if(warnLevel>0) cout << "WARNING (ContactFlexible::setDamping) Deprecated function and only valid for linear contact law. Use setDampingFunction instead." << endl; delete d_fun; d_fun = new FuncLinear(Vec(1,INIT,-d_),Vec(1,INIT,0.));}
  inline void ContactFlexible::setDampingFunction(MBSim::DataInterfaceBase *d_fun_) {delete d_fun; d_fun = d_fun_;}
  inline void ContactFlexible::setMarginalVelocity(double v) {gdT_grenz = v;}
}

#endif /* _CONTACT_FLEXIBLE_H_ */

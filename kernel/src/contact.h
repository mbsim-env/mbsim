/* Copyright (C) 2004-2006  Martin F�rg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#ifndef _CONTACT_H_
#define _CONTACT_H_

#include "link.h"
#include "contour.h"

namespace MBSim {

  class ContactKinematics;

  /*! \brief Class for contacts
   *
   * Basis class for Contacts between Contours, mainly implementing geometrical informations of ContactPairings
   */
  class Contact: public LinkContour {
    public:
      /*! Constructor setting set-valued or functional contact laws	*/      
      Contact(const string &name, bool setValued);
      /*! Clone constructor with new name, but same parameters */
      Contact(const Contact *master, const string &name_);
	  /*! Destructor */
      virtual ~Contact();
	  /*! Calculates dimension of needed force, gap vectors */
      void calcSize();
      /*! Initialises the needed contour description */
      void init();
      /*! Define whether HitSpheres are tested or ignored */
      void connectHitSpheres(Contour *contour1, Contour* contour2);
      /*! Connect two Contours and select the contact kinematics */
      void connect(Contour *contour1, Contour* contour2);
	  /*! First step of calculating the contact kinematics */
      void updateStage1(double t);
      /*! Second step of calculating the contact kinematics */
      void updateStage2(double t);
      /*! Define force directions and evaluate kinetical values in updateStage2 */
      virtual void updateKinetics(double t) = 0;
	  /*! Set number of friction directions: 0,1,2 */
      void setFrictionDirections(int nFric_);
      /*! Get number of friction directions */
      int getFrictionDirections() const;
      /*! Set friction coefficent */
      void setFrictionCoefficient(double mue_);
      /*! Get friction coefficent */
      double getFrictionCoefficient() const;	  
      /*! Set friction coefficient function with norm of relativ tangential velocity as argument */
      void setFrictionCoefficientFunction(DataInterfaceBase *mue_fun_);
      /*! Get friction coefficient function */
      const DataInterfaceBase* getFrictionCoefficientFunction() const;
      /*! Set kinematical relationships between contact partners */
      void setContactKinematics(ContactKinematics* ck);
      using LinkContour::connect;
      
    protected:
      /** friction coefficient function */
      DataInterfaceBase *mue_fun;
      /** index for tangential directions in projection matrices */
      Index iT;
      /** number of friction directions: 0 = frictionless, 1 = planar friction, 2 = spatial friction */
      int nFric;
	  /** kinematical description of the contact relationship between two partners */
	  ContactKinematics *contactKinematics;
	  /*! Tests, if a contact is closed (=active) or not */
      virtual void checkActive();
  };

  inline void Contact::setFrictionDirections(int nFric_) {nFric = nFric_;}
  inline int Contact::getFrictionDirections() const {return nFric;}
  inline void Contact::setFrictionCoefficient(double mue_) {cout << "Warning (Contact::setFrictionCoefficient): Deprecated function and only valid for Coulomb friction. Use setFrictionCoefficientFunction instead." << endl; delete mue_fun; mue_fun = new FuncConst(Vec(1,INIT,mue_));}
  inline double Contact::getFrictionCoefficient() const {cout << "Warning (Contact::getFrictionCoefficient): Deprecated function and only valid for Coulomb friction. Use getFrictionCoefficientFunction instead." << endl; return ((*mue_fun)(0.))(0);}
  inline void Contact::setFrictionCoefficientFunction(DataInterfaceBase *mue_fun_) {delete mue_fun; mue_fun = mue_fun_;}
  inline const DataInterfaceBase* Contact::getFrictionCoefficientFunction() const {return mue_fun;}
  inline void Contact::setContactKinematics(ContactKinematics* ck) {contactKinematics = ck;}
}

#endif /* _CONTACT_H_ */

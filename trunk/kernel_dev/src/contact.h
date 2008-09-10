/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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
//#include "hitsphere_link.h"  \TODO Warum brauchen wir das nicht?
#include "contour.h"


namespace MBSim {

  class ContactKinematics;

  /*! \brief Class for contacts
   *
   * Basis class for Contacts between Contours, mainly implementing geometrical informations of ContactPairings
   *
   * */
  class Contact: public Link {

    protected:
      /** friction coefficinet */
      double mue;
      /** index for tangential directions in projection matrices */
      Index iT;

      /** number of friction directions: 0 = frictionless, 1 = planar friction, 2 = spatial friction */
      int nFric;

      virtual void checkActive();

      ContactKinematics *contactKinematics;

    public:
      /*!
	\param name name of Contact
	\param setValued true, if force law is set-valued, else false for functional law
	*/      
      Contact(const string &name, bool setValued);

      /*! clone constructor with new name ! same parameters ! */
      Contact(const Contact *master, const string &name_);

      virtual ~Contact();

      void calcSize();
      /*geerbt*/
      void init();

      /*! define wether HitSpheres are tested or ignored
      */
      void connectHitSpheres(Contour *contour1, Contour* contour2);

      /*! connect two Contour s
	\param contour1 first contour
	\param contour2 second contour
	*/
      void connect(Contour *contour1, Contour* contour2);

      void updateStage1(double t);
      void updateStage2(double t);

      /*! define force dircetions and evaluate kinematical values */
      virtual void updateKinetics(double t) = 0;

      void setFrictionDirections(int nFric_) {nFric = nFric_;}
      int getFrictionDirections() const {return nFric;}

      /*! set one friction coefficinet for directions tangential to contact */
      void setFrictionCoefficient(double mue_) {mue = mue_;}
      /*! get friction coefficinet */
      double getFrictionCoefficient() {return mue;}

      void setContactKinematics(ContactKinematics* ck) {contactKinematics = ck;}
      string getType() const {return "Contact";}

      using Link::connect;
  };

}

#endif

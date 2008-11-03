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
      //bool hca, nhca, closed, sticking;
      unsigned int gActive, gdActive[2];
      unsigned int gActive0, gdActive0[2];

      /** index for tangential directions in projection matrices */
      Index iT;

      /** number of friction directions: 0 = frictionless, 1 = planar friction, 2 = spatial friction */
      //int nFric;

      ContactKinematics *contactKinematics;

    public:
      /*!
	\param name name of Contact
	\param setValued true, if force law is set-valued, else false for functional law
	*/      
      Contact(const string &name, bool setValued);

      virtual ~Contact();

      void calcxSize();

      void calclaSize();
      void calcgSize();
      void calcgdSize();
      void calcrFactorSize();

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

      void updateg(double t);
      void updategd(double t);
      void updater(double t);

      void updateWRef(const Mat &ref);
      void updateVRef(const Mat &ref);

      /*! set one friction coefficinet for directions tangential to contact */
      //void setFrictionCharacteristics(UserFunction *fmu_) {fmu = fmu_;}
      //void setFrictionCoefficient(double mu_) {mu = mu_;}
      ///*! get friction coefficinet */
      //double getFrictionCoefficient() {return mu;}

      virtual int getFrictionDirections() {return 0;}
      void setContactKinematics(ContactKinematics* ck) {contactKinematics = ck;}
      string getType() const {return "Contact";}

      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);


      //bool activeConstraintsChanged();
      bool gActiveChanged();
      //bool activeHolonomicConstraintsChanged();
      //bool activeNonHolonomicConstraintsChanged();

      using Link::connect;
  };

}

#endif

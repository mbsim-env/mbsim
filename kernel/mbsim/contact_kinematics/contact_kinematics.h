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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTACT_KINEMATICS_H_
#define _CONTACT_KINEMATICS_H_

#include "mbsim/links/single_contact.h"
#include "fmatvec/fmatvec.h"
#include "fmatvec/atom.h"
#include <vector>

namespace MBSim {

  class ContourFrame;
  class Contour;
  template<typename Sig> class Function;

  /** 
   * \brief basic class for contact kinematical calculations
   * \author Martin Foerg
   * \date 2009-03-18 some comments (Thorsten Schindler)
   * \date 2009-04-02 velocity part deleted (Thorsten Schindler)
   * \date 2009-07-28 updates are pure virtual (Thorsten Schindler)
   */
  class ContactKinematics : virtual public fmatvec::Atom {
    public:
      /**
       * \brief constructor
       */
      ContactKinematics(int maxNumContacts_=1) : maxNumContacts(maxNumContacts_) { }

      /**
       * \brief destructor
       */
      ~ContactKinematics() override = default;

      /* INTERFACE FOR DERIVED CLASSES */
      /** 
       * \brief treats ordering of contours 
       * \param contour vector
       */
      virtual void assignContours(const std::vector<Contour*> &contour_) { contour = contour_; }

      virtual void calcisSize() { isSize = 0; }
      void setisInd(int ind) { isInd = ind; }
      int getisSize() { return isSize; }
      virtual void updateInternalStateRef(fmatvec::Vec &curisParent, fmatvec::Vec &nextisParent) {
        curis.ref(curisParent, fmatvec::RangeV(isInd,isInd+isSize-1));
        nextis.ref(nextisParent, fmatvec::RangeV(isInd,isInd+isSize-1));
      }

      virtual void search() { }

      /**
       * \brief compute contact kinematics for all contacts
       * \param contact vector of all contacts
       * \param i index of the contact point that should be updated
       */
      virtual void updateg(std::vector<SingleContact> &contact);

      /**
       * \brief compute contact kinematics for a single contact
       * \param conctact single contact
       * \param i index of the contact that should be updated
       */
      virtual void updateg(SingleContact &contact, int i=0) { }

      /**
       * \brief compute contact kinematics on acceleration level for all contacts
       * \param conctact single contact
       * \param i index of the contact that should be updated
       */
      virtual void updatewb(std::vector<SingleContact> &contact) { for(int i=0; i<maxNumContacts; i++) updatewb(contact[i],i); }

      /**
       * \brief compute contact kinematics on acceleration level for a single contact
       * \param conctact single contact
       * \param i index of the contact that should be updated
       */
      virtual void updatewb(SingleContact &contact, int i=0);
      
      /** 
       * \brief treats ordering of contours
       * \param first contour
       * \param second contour
       */
      void assignContours(Contour *contour1, Contour *contour2) { std::vector<Contour*> c; c.push_back(contour1);c.push_back(contour2); assignContours(c); }

      /**
       * \return maximum number of contacts
       */
      int getMaximumNumberOfContacts() const { return maxNumContacts; }

      void setGlobalSearch(bool gS_=true) { gS = gS_; }
      void setInitialGlobalSearch(bool iGS_=true) { iGS = iGS_; }

      /**
       * \brief set initial guess for root-finding
       */
      virtual void setInitialGuess(const fmatvec::MatV &zeta0_) { }

      /**
       * \brief set tolerance for root-finding
       */
      void setTolerance(double tol_) { tol = tol_; }

      /**
       * \brief set maximum number of contacts
       */
      void setMaximumNumberOfContacts(int maxNumContacts_) { maxNumContacts = maxNumContacts_; }

      static std::vector<double> searchPossibleContactPoints(Function<double(double)> *func, double eta, const std::vector<double> &nodes, double tol);

      static std::vector<double> searchPossibleContactPoints(Function<fmatvec::Vec(fmatvec::Vec)> *func, int i, fmatvec::Vec &zeta, const std::vector<double> &nodes, double tol);

    protected:
      /**
       * \brief tolerance for root-finding
       */
      double tol{1e-10};

      /**
       * \brief maximum number of contacts
       */
      int maxNumContacts{1};

      fmatvec::Vec curis, nextis;
      int isInd { 0 };
      int isSize { 0 };

      std::vector<Contour*> contour;

      bool gS{false};
      bool iGS{false};
  };

}

#endif /* _CONTACT_KINEMATICS_H_ */

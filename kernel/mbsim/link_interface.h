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

#ifndef _LINK_INTERFACE_H_
#define _LINK_INTERFACE_H_

namespace MBSim {

  /*!
   * all Link%s use this common interface regardless whether single- or set-valued
   * / smooth or non-smooth interactions are defined
   * \brief interface for links
   * \author Martin Foerg
   * \date 2009-03-09 some comments (Thorsten Schindler)
   * \date 2009-03-26 enhanced comments (Roland Zander)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2010-07-06 LinkStatus added (Robert Huber)
   */
  class LinkInterface {
    public:
      /*!
       * \brief constructor 
       */
      LinkInterface() {}

      /*!
       * \brief destructor
       */
      virtual ~LinkInterface() {}

      /*!
       * \brief updates nonlinear relative acceleration term
       * \param simulation time
       */
      virtual void updatewb(double t) = 0;

      /*!
       * \brief updates JACOBIAN matrix \f$\vW\f$ between Lagrangian multipliers and generalised velocities
       * \param simulation time
       */
      virtual void updateW(double t) = 0;

      /*!
       * for event driven integration, \f$\vW\f$ can be condensed regarding dependent
       * tangential contacts, where \f$\Lambda_N\f$ also occures in evaluation of tangential
       * force when sliding; in this case, \f$\vV\f$ might hold less columns than \f$\vW\f$
       * \brief updates condensed JACOBIAN matrix between condensed Lagrangian multipliers and generalised velocities
       * \param simulation time
       */
      virtual void updateV(double t) = 0;

      /*!
       * for links holding smooth contributions \f$\vF\f$, the respective forces are projected
       * into the minimal coordinate representation of the associated Body%s using the
       * JACOBIAN matrices \f$\vJ\f$.
       * \f[ \vh = \vJ\vF \f]
       * The JACOBIAN is provided by the connected Frame%s (which might be
       * user-defined for Joint%s or internally defined for Contact%s).
       * \brief update smooth link force law
       * \param simulation time
       */
      virtual void updateh(double t) = 0;

      /**
       * \brief updates Jacobian for implicit integration regarding state
       * \param simulation time
       */
      virtual void updatedhdz(double t) = 0;
      
      /*!
       * \brief update relative distance
       * \param simulation time
       */
      virtual void updateg(double t) = 0;

      /*!
       * \brief update relative velocity
       * \param simulation time
       *
       * compute normal and tangential relative velocities, velocity and angular velocity of possible contact point if necessary
       */
      virtual void updategd(double t) = 0;

      /*!
       * \brief update stop vector (root functions for event driven integration)
       * \param simulation time
       */
      virtual void updateStopVector(double t) = 0;
        
      /*!
       * \brief update status of set valued links with piecewise link equations (which piece is valid)
       */ 
      virtual void updateLinkStatus(double t) = 0;

      /*!
       * \brief update acceleration description of frames and contours
       * \param simulation time
       */
      virtual void updateJacobians(double t) = 0;
  };

}

#endif /* _LINK_INTERFACE_H_ */


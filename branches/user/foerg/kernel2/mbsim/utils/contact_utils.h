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
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_UTILS_H_
#define _CONTACT_UTILS_H_

#include "mbsim/contact.h"
#include "mbsim/contour.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/dynamic_system_solver.h"

#include <vector>

namespace MBSim {

  /**
   * \brief computes azimuthal angle
   * \author Martin Foerg
   * \date some comments (Thorsten Schindler)
   */
  double computeAngleOnUnitCircle(const fmatvec::FVec& r);

  /**
   * \brief computes azimuthal and polar angles
   * \author Martin Foerg
   * \date some comments (Thorsten Schindler)
   */
  fmatvec::Vector<fmatvec::GeneralFixed<2,1>, double> computeAnglesOnUnitSphere(const fmatvec::FVec& r);

  /**
   * \brief defines contact kinematics between two contours
   * \author Martin Foerg
   * \date 2009-07-14 some comments (Thorsten Schindler)
   * \date 2010-11-05 Interface changed (Markus Schneider)
   */
  ContactKinematics* findContactPairingRigidRigid(const char* contour0, const char* contour1);

  /*!
   * \brief apply contact between ContourInterpolation surfaces, using node-to-surface pairings, with both as master
   * \author Roland Zander
   * \date 2009-07-14 some comments (Thorsten Schindler)
   */
  template <class T>
    void ContactContourInterpolation(DynamicSystemSolver *ds, T *contact, ContourInterpolation *contour0, ContourInterpolation *contour1) {
      ContactContourInterpolation(ds,contact,contour0,contour1,0);
    }

  /*!
   * \brief apply contact between ContourInterploation surfaces, using node-to-surface pairings, defining master contour 1 or 2, default 0 for both
   * \author Roland Zander
   * \date 2009-07-14 some comments (Thorsten Schindler)
   */
  template <class T>
    void ContactContourInterpolation(DynamicSystemSolver *ds, const T *contact, ContourInterpolation *contour0, ContourInterpolation *contour1, int master) {
      ContourInterpolation *contour[2];
      contour[0] = contour0;
      contour[1] = contour1;
      std::string contactName = contact->getName();

      int cStart, cEnd;
      switch(master) {
        case 0: cStart = 0; cEnd = 1; break;
        case 1: cStart = 1; cEnd = 1; break;
        case 2: cStart = 0; cEnd = 0; break;
      }

      for(int c = cStart;c<cEnd+1;c++) { // loop with respect to contours
        int numberOfPoints = contour[c]->getNPoints();
        char contourName;
        switch(c) {
          case 0: contourName = 'A';break;
          case 1: contourName = 'B';break;
        }
        for(int i = 0;i<numberOfPoints; i++) { // use all points
          std::stringstream number;
          std::string name = contactName + number.str();

          T *newContact = new T( contact , name );

          newContact->connect(contour[c]->getPoint(i),contour[1-c]);
          ds->addLink(newContact);
        }
      }
    }

}

#endif /* _CONTACT_UTILS_H_ */


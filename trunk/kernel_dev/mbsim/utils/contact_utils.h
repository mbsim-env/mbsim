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
 * Contact: mfoerg@users.berlios.de
 *          rzander@users.berlios.de
 */

#ifndef _CONTACT_UTILS_H_
#define _CONTACT_UTILS_H_

#include "mbsim/contact.h"
#include "mbsim/contour.h"
#include "mbsim/dynamic_system_solver.h"

#include <vector>

namespace MBSim {

  double computeAngleOnUnitCircle(const fmatvec::Vec& r);
  fmatvec::Vec computeAnglesOnUnitSphere(const fmatvec::Vec& r);

  ContactKinematics* findContactPairing(Contour *contour0, Contour *contour1);

  /*!
    apply contact between ContourInterploation surfaces, using node-to-surface pairings, with both as master
    */
  template <class T>
    void ContactContourInterpolation(DynamicSystemSolver *ds, T *contact, ContourInterpolation *contour0, ContourInterpolation *contour1) {
      ContactContourInterpolation(ds,contact,contour0,contour1,0);
    }
  /*!
    apply contact between ContourInterploation surfaces, using node-to-surface pairings, defining master contour 1 or 2, default 0 for both
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

      // die beiden Contouren abklappern
      for(int c = cStart;c<cEnd+1;c++) {
        int numberOfPoints = contour[c]->getNPoints();
        //	int intWidth = static_cast<int>(round(log(static_cast<double>(numberOfPoints))));
        char contourName;
        switch(c) {
          case 0: contourName = 'A';break;
          case 1: contourName = 'B';break;
        }

        // alle Punkte verwenden
        for(int i = 0;i<numberOfPoints; i++) {
          /* 	    cout << "\n-------------\nprocessing point " << contour[c]->getPoint(i)->getName()  << endl; */

          // Namensgebung
          std::stringstream number;
          /* 	    number << "." << contourName << "." << contour[c]->getPoint(i)->getName(); */
          std::string name = contactName + number.str();

          // von Vorlage abschreiben
          T *newContact = new T( contact , name );//

          // Verbinden
          newContact->connect(contour[c]->getPoint(i),contour[1-c]);
          ds->addLink(newContact);

          /* 	    cout << "added contact \"" << newContact->getName() << "\"" << endl; */
        }
      }
    }

}

#endif


/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef POINT_FLEXIBLEBAND_H_
#define POINT_FLEXIBLEBAND_H_

#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/mbsim_event.h"
#include "mbsimFlexibleBody/contact_kinematics/point_flexibleband.h"
#include "mbsim/contour.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/contours/point.h"
#include "mbsim/functions_contact.h"

namespace MBSim {
  class Point;
}

namespace MBSimFlexibleBody {

  template<class Col>
  class FlexibleBand;

  /**
   * \brief pairing point to flexible band
   * \author Thorsten Schindler
   * \author Roland Zander
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2010-04-15 bug fixed: different sign in Wd (Thomas Cebulla)
   */
  template<class Col>
  class ContactKinematicsPointFlexibleBand : public MBSim::ContactKinematics {
    public:
      /**
       * \brief constructor
       */
      ContactKinematicsPointFlexibleBand();
      
      /**
       * \brief destructor
       */
      virtual ~ContactKinematicsPointFlexibleBand();

      /* INHERITED INTERFACE OF CONTACTKINEAMTICS */
      virtual void assignContours(const std::vector<MBSim::Contour*>& contour);
      virtual void updateg(fmatvec::Vec& g, MBSim::ContourPointData *cpData);   
      virtual void updatewb(fmatvec::Vec& wb, const fmatvec::Vec &g, MBSim::ContourPointData *cpData) { throw MBSim::MBSimError("ERROR (ContactKinematicsPointFlexibleBand::updatewb): not implemented!"); }   
      /***************************************************/

    private:
      /** 
       * \brief contour index 
       */
      int ipoint, icontour;

      /** 
       * \brief contour classes 
       */
      MBSim::Point *point;
      FlexibleBand<Col> *band;
  };

  template <class Col>
  inline ContactKinematicsPointFlexibleBand<Col>::ContactKinematicsPointFlexibleBand() :
      ContactKinematics(), ipoint(0), icontour(0), point(0), band(0) {
  }

  template<class Col>
  inline ContactKinematicsPointFlexibleBand<Col>::~ContactKinematicsPointFlexibleBand() {
  }

  template<class Col>
  inline void ContactKinematicsPointFlexibleBand<Col>::assignContours(const std::vector<MBSim::Contour*>& contour) {
    if (dynamic_cast<MBSim::Point*>(contour[0])) {
      ipoint = 0;
      icontour = 1;
      point = static_cast<MBSim::Point*>(contour[0]);
      band = static_cast<FlexibleBand<Col>*>(contour[1]);
    }
    else {
      ipoint = 1;
      icontour = 0;
      point = static_cast<MBSim::Point*>(contour[1]);
      band = static_cast<FlexibleBand<Col>*>(contour[0]);
    }
  }

  template<class Col>
  inline void ContactKinematicsPointFlexibleBand<Col>::updateg(fmatvec::Vec& g, MBSim::ContourPointData *cpData) {
    cpData[ipoint].getFrameOfReference().setPosition(point->getFrame()->getPosition()); // position of point

    MBSim::FuncPairContour1sPoint *func = new MBSim::FuncPairContour1sPoint(point, band); // root function for searching contact parameters
    MBSim::Contact1sSearch search(func);
    search.setNodes(band->getNodes()); // defining search areas for contacts

    if (cpData[icontour].getLagrangeParameterPosition().size() != 0) { // select start value from last search
      search.setInitialValue(cpData[icontour].getLagrangeParameterPosition()(0));
    }
    else { // define start search with regula falsi
      search.setSearchAll(true);
      cpData[icontour].getLagrangeParameterPosition() = fmatvec::VecV(2, fmatvec::NONINIT);
    }

    cpData[icontour].getLagrangeParameterPosition()(0) = search.slv(); // get contact parameter of neutral fibre
    cpData[icontour].getLagrangeParameterPosition()(1) = 0.;

    if (cpData[icontour].getLagrangeParameterPosition()(0) < band->getAlphaStart() || cpData[icontour].getLagrangeParameterPosition()(0) > band->getAlphaEnd())
      g(0) = 1.;
    else {
      band->updateKinematicsForFrame(cpData[icontour], MBSim::position_cosy);
      fmatvec::Vec3 Wd = cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour].getFrameOfReference().getPosition();
      fmatvec::Vec3 Wb = cpData[icontour].getFrameOfReference().getOrientation().col(2);
      cpData[icontour].getLagrangeParameterPosition()(1) = Wb.T() * Wd; // get contact parameter of second tangential direction

      double width = band->getWidth();
      if (cpData[icontour].getLagrangeParameterPosition()(1) > 0.5 * width || -cpData[icontour].getLagrangeParameterPosition()(1) > 0.5 * width)
        g(0) = 1.;
      else { // calculate the normal distance
        cpData[icontour].getFrameOfReference().getPosition() += cpData[icontour].getLagrangeParameterPosition()(1) * Wb;
        cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[icontour].getFrameOfReference().getOrientation().col(0));
        cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[icontour].getFrameOfReference().getOrientation().col(1));
        cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[icontour].getFrameOfReference().getOrientation().col(2));
        g(0) = cpData[icontour].getFrameOfReference().getOrientation().col(0).T() * (cpData[ipoint].getFrameOfReference().getPosition() - cpData[icontour].getFrameOfReference().getPosition());
      }
    }
    delete func;
  }

}

#endif /* POINT_FLEXIBLEBAND_H_ */


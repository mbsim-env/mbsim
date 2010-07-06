/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_
#define MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_

#include "mbsim/contours/circle.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
//#include "mbsim/utils/function.h"
#include "mbsim/functions_contact.h"

namespace MBSimFlexibleBody {

  /*!
   * \brief root function for pairing Circle and NurbsDisk2s
   * \author Kilian Grundl
   * \date 2009-10-06 initial commit (Thorsten Schindler)
   */
  class FuncPairCircleNurbsDisk2s : public MBSim::DistanceFunction<double,double> {
    public:
      /**
       * \brief constructor
       * \param circle
       * \param nurbsdisk
       */
      FuncPairCircleNurbsDisk2s(MBSim::Circle* circle_, NurbsDisk2s* nurbsdisk_) : nurbsdisk(nurbsdisk_), circle(circle_) {}

      /* INHERITED INTERFACE OF DISTANCEFUNCTION */
      double operator()(const double &alpha, const void * =NULL) {
        //Parameters of the AWK of the nurbs disk and the circle
        fmatvec::SqrMat AWK_disk   = nurbsdisk->getFrame()->getOrientation();
        fmatvec::SqrMat AWK_circle = circle->getFrame()->getOrientation();

        //Point on the Circle
        fmatvec::Vec WP_circle(3,fmatvec::INIT,0.);  //world-coordinates of the point on the circle
        WP_circle(0) = cos(alpha);  
        WP_circle(1) = sin(alpha);
        WP_circle = circle->getFrame()->getPosition() + circle->getRadius() * AWK_circle * WP_circle;

        //derivatives of a point on the circle in world-coordinates with respect to the circle-parameter alpha
        fmatvec::Vec dWP_circle(3,fmatvec::INIT, 0.);
        dWP_circle(0) = -sin(alpha);  
        dWP_circle(1) = cos(alpha);
        fmatvec::Vec circle_tangent = circle->getRadius() * AWK_circle * dWP_circle; //not normalised tangent on the circle

        //compute radial and azimuthal nurbsdisk-coordinates out of alpha (saved in the LagrangeParameterPosition)
        MBSim::ContourPointData cp_nurbsdisk;
        cp_nurbsdisk.getLagrangeParameterPosition() = nurbsdisk->transformCW( AWK_disk.T() * (WP_circle - nurbsdisk->getFrame()->getPosition()) ); // position of the point in the cylinder-coordinates of the disk

        //get the position and the derivatives on the disk 
        nurbsdisk->updateKinematicsForFrame(cp_nurbsdisk,MBSim::firstTangent); 

        //compute the derivates of the radial and the azimuthal coordinates with respect to alpha
        fmatvec::SqrMat A_inv(3,fmatvec::EYE);
        A_inv(0,0)=  cos(cp_nurbsdisk.getLagrangeParameterPosition()(1)); 
        A_inv(0,1)=  sin(cp_nurbsdisk.getLagrangeParameterPosition()(1)); 
        A_inv(1,0)= -sin(cp_nurbsdisk.getLagrangeParameterPosition()(1)) / cp_nurbsdisk.getLagrangeParameterPosition()(0); 
        A_inv(1,1)=  cos(cp_nurbsdisk.getLagrangeParameterPosition()(1)) / cp_nurbsdisk.getLagrangeParameterPosition()(0);
        fmatvec::Vec drphidalpha = A_inv * AWK_disk.T()* circle_tangent; // AWK_disk * A_inv * trans(AWK_disk)* circle_tangent CHANGED

        //compution of the single elements in the function
        fmatvec::Vec nurbs_radial_tangent    = cp_nurbsdisk.getFrameOfReference().getOrientation().col(1);
        fmatvec::Vec nurbs_azimuthal_tangent = cp_nurbsdisk.getFrameOfReference().getOrientation().col(2);

        return nurbsdisk->getFrame()->getOrientation().col(2).T() * (circle_tangent - (nurbs_radial_tangent *  drphidalpha(0)+ nurbs_azimuthal_tangent * drphidalpha(1)));
      }

      fmatvec::Vec computeWrD(const double &alpha) {
        //point on the circle
        fmatvec::Vec WP_circle(3,fmatvec::INIT,0.);
        WP_circle(0) = cos(alpha); 
        WP_circle(1) = sin(alpha);
        WP_circle = circle->getFrame()->getPosition() + circle->getRadius() * circle->getFrame()->getOrientation() * WP_circle;    

        //get the position on the nurbsdisk
        MBSim::ContourPointData cp_nurbsdisk;
        cp_nurbsdisk.getLagrangeParameterPosition() = nurbsdisk->transformCW(nurbsdisk->getFrame()->getOrientation().T()*(WP_circle - nurbsdisk->getFrame()->getPosition())); // position of the point in the cylinder-coordinates of the disk
        nurbsdisk->updateKinematicsForFrame(cp_nurbsdisk,MBSim::position);
        fmatvec::Vec WP_nurbsdisk = cp_nurbsdisk.getFrameOfReference().getPosition();

        return WP_circle - WP_nurbsdisk;
      }
      /***************************************************/

    private:
      /**
       * \brief contours
       */
      NurbsDisk2s *nurbsdisk;
      MBSim::Circle *circle;
  };

}

#endif /* MBSIMFLEXIBLEBODY_FUNCTIONS_CONTACT_H_ */


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
 * Contact: thschindler@users.berlios.de
 */

#ifndef NURBSDISK2S_H_
#define NURBSDISK2S_H_

#include "fmatvec.h"
#include "mbsim/contour.h"
#ifdef HAVE_NURBS
#include "nurbsS.h"
#include "vector.h"
using namespace PLib;
#endif

namespace MBSim {
 
  /*!  
   * \brief 2s flexible
   * \author Kilian Grundl
   * \author Raphael Missel
   * \author Thorsten Schindler
   * \date 2009-05-22 initial commit (Grundl / Missel / Schindler)
   * \todo HAVE_NURBS has to be set correctly
   */
  class NurbsDisk2s : public Contour2s {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      NurbsDisk2s(const std::string &name) : Contour2s(name) {}

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "NurbsDisk2s"; }
      /***************************************************/

      /* INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(ContourPointData &cp);
      virtual void computeRootFunctionFirstTangent(ContourPointData &cp);
      virtual void computeRootFunctionNormal(ContourPointData &cp);
      virtual void computeRootFunctionSecondTangent(ContourPointData &cp);
      /***************************************************/

      void init(const int dU, const int dV, const int nBr, const int nBj, const double &innerRadius, const double &outerRadius, const int flagN);

      fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);

      void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      /*! computes derivates of the surface  - 1.col:deg-times in radial direction and 2.col: deg-times in azimuthal-direction*/
      fmatvec::Mat computeDirectionalDerivatives(const double &radius, const double &phi, const int &deg);
      /*! computes the curvature on the surface */
      fmatvec::Mat computeCurvatures(const double &radius, const double &phi);

      /*! computes the U vector of the surface for a closed Interpolation */
      void computeUVec(const int NbPts); 

      /*! computes the V-vector of the surface for an open Interpolation */
      void computeVVec(const int NbPts);

      /*! interpolates the surface with node-data from body*/
      void computeSurface();

      /*! interpolates the velocities of the surface with the node-data from the body*/
      void computeSurfaceVelocities();

      /*! returns a Vector of a Controlpoint */
      fmatvec::Vec getCtrlPts(const int u, const int v);

      /*! returns the U-Vector of the Surface (azimuthal direction) */
      fmatvec::Vec getUVec();

      /*! returns the V-Vector of the Surface (radial direction)*/
      fmatvec::Vec getVVec();

      /*! checks, whether the input radius is inside the bounds or the angle is between 0 and 2PI*/
      int testInsideBounds(const fmatvec::Vec &s);

      /*! TESTING computes the error between two Vectors */
      double computeError(const fmatvec::Vec &Vec1, const fmatvec::Vec &Vec2);
      /*! TESTING displays a Vector with cout */
      void dispVec(const fmatvec::Vec& Vec);

    protected:
      int nj, nr;
      int degU, degV;
      int flagNormal;
      double Ri, Ra;

#ifdef HAVE_NURBS
      /** Interpolated surface of the contour */
      PlNurbsSurfaced *Surface; //a homogenous Surface with accuracy of calculation of double (HSurface<double,3>)
      /** Interpolated velocities of the surface-points */
      PlNurbsSurfaced *SurfaceVelocities; 
      /** knotVectors, used for the U und V coordinates of the surface */
      Vector<double> *uvec; //nurbs++ needs this vector 
      Vector<double> *uVec; //knot-Vector for azimuthal direction 
      Vector<double> *vvec; //nurbs++ needs this vector
      Vector<double> *vVec; //knot-Vector for radial direction
#endif
  };

}

#endif /* NURBSDISK2S_H_ */


/* Copyright (C) 2004-2015 MBSim Development Team
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
 *          rzander@users.berlios.de
 */

#ifndef BODY_FLEXIBLE_1S_01_TORSION_H_
#define BODY_FLEXIBLE_1S_01_TORSION_H_

#include "mbsimFlexibleBody/flexible_body.h"

namespace MBSimFlexibleBody {

//  class Contour1sFlexible;

  /**
   * \brief torsional axis with polynomial ansatz of order \f$n\f$
   * \author Roland Zander
   * \author Rainer Britz
   */
  class BodyFlexible1s01Torsion : public FlexibleBodyContinuum<double> {
    protected:
      int n;
      double E, rho, A, I, l;
      //    Vec JGeneralized(const double &s); // nur zur Kompatibilitaet

      fmatvec::Vec sTangent;
      fmatvec::Mat Wt;
      fmatvec::Vec Wn, CrOC, CvC;

      // KOS-Definition und Lage des ersten Knoten im Weltsystem
      //    Mat Jges;
      fmatvec::Vec Axis; /* Enspricht der neutralen Faser */
      fmatvec::Vec WrON00, WrON0;

      void init(InitStage stage);
      void initMatrizes();

      void updateStateDependentVariables(double t);
      void updatePorts(double t);
      //void sumUpForceElements(double t);
      //void updateW(double t);

      void updateh(double t);

    public:

      BodyFlexible1s01Torsion(const std::string &name);
      /*! set number of polynom shape functions
       */
      void setNumberShapeFunctions(int n_);
      /*! set cross-sectional area
       */
      void setCrossSectionalArea(double A_) {
        A = A_;
      }
      /*! set moment of inertia
       */
      void setMomentInertia(double I_) {
        I = I_;
      }
      /*! set E-modul
       */
      void setEModul(double E_) {
        E = E_;
      }
      /*! set mass density
       */
      void setDensity(double rho_) {
        rho = rho_;
      }
      /*! set length of axis
       */
      void setLength(double l_) {
        l = l_;
      }
      /*! set initial rotational velocity
       */
      void setInitialRotationVelocity(double omega0) {
        u0(1) = omega0;
      }
      /*! \attic set initial rotational velocity, method kept for compatibility
       */
      void setInitialRotation(double omega0) {
        setInitialRotationVelocity(omega0);
      }

      /*     void addPort(const string &name, const double &S); */
      /*     using BodyFlexible1s::addPort; */

      fmatvec::Mat computeJacobianMatrix(const MBSim::ContourPointData &S_); // virtual of body_flexible

      fmatvec::Mat computeWt(const MBSim::ContourPointData &S_) {
        return Axis;
      }
      fmatvec::Vec computeWn(const MBSim::ContourPointData &S_) {
        return fmatvec::Vec(3);
      }
      fmatvec::Vec computeWrOC(const MBSim::ContourPointData &S_) {
        return WrON00 + S_.alpha(0) * Axis;
      }
      fmatvec::Vec computeWvC(const MBSim::ContourPointData &S_) {
        return fmatvec::Vec(3);
      }

      /*! \return true
       */
      bool hasConstMass() const {
        return true;
      }

      /*! NULL-function, since mass matrix is constant*/
      void facLLM() {
      }

      /* Jacobi Matrix der Rotation */
      void setJR(const fmatvec::Vec &JR_) {
        assert(JR_.cols() == 1);
        JR = JR_.copy();
        Axis = JR_;
      }
      ;

      // Lage des Ursprungs des Wellensystems im WKOS 
      // Per Default liegt es im Ursprung des WKOS
      void setWrON00(const fmatvec::Vec &WrON00_) {
        WrON00 = WrON00_;
      }

      /** Steifigkeitsmatrix */
      fmatvec::SymMat K;

  };

}

#endif

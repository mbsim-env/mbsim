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
 * Created on: Jul 30, 2013
 * Contact: kilian.grundl@gmail.com
 */

#ifndef FINITE_ELEMENT_1S_REFERENCE_CURVE_H_
#define FINITE_ELEMENT_1S_REFERENCE_CURVE_H_

#include <mbsimFlexibleBody/flexible_body/flexible_body_1S_reference_curve.h>

#include <mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_reference_curve.h>

#include <mbsim/discretization_interface.h>

#include <mbsim/functions/function.h>

namespace MBSimFlexibleBody {

  /*!
   * \brief Finite-Element class for the body FlexibleBody1SReferenceCurve
   */
  class FlexibleBody1SReferenceCurveFE : public MBSim::DiscretizationInterface {
      friend class funcPTP;
      friend class funcPTdPdxi;
      friend class funcPTdPdt;
      friend class funcPTdPdqk;
      friend class funcForWgamma;
      friend class funcForWb;

    public:
      /*!
       * \brief standard constructor
       */
      FlexibleBody1SReferenceCurveFE(FlexibleBody1SReferenceCurve * parent, int eleNo, Vec2 alpha, int order, int nodeDofs);

      virtual ~FlexibleBody1SReferenceCurveFE();

      /*!
       * \brief initialize the FE
       */
      virtual void init(MBSim::Element::InitStage stage);
      virtual int getqSize() const {
        throw MBSim::MBSimError("Not implemented" + string(__func__));
      }

      virtual fmatvec::Vec getq() const {
        return parent->qElement[element];
      }

      virtual int getuSize() const {
        throw MBSim::MBSimError("Not implemented" + string(__func__));
      }

      virtual const fmatvec::SymMat& getM() const {
        return M;
      }
      virtual const fmatvec::Vec& geth() const {
        return h;
      }

      virtual const fmatvec::SqrMat& getdhdq() const {
        throw MBSim::MBSimError("Not implemented" + string(__func__));
      }
      virtual const fmatvec::SqrMat& getdhdu() const {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual void computeM(const fmatvec::Vec& q);
      virtual void computeh(const fmatvec::Vec& q, const fmatvec::Vec& u);
      virtual void computedhdz(const fmatvec::Vec& q, const fmatvec::Vec& u) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual double computeKineticEnergy(const fmatvec::Vec& q, const fmatvec::Vec& u) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual double computeGravitationalEnergy(const fmatvec::Vec& q) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual double computeElasticEnergy(const fmatvec::Vec& q) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual fmatvec::Vec computePosition(const fmatvec::Vec& q, const MBSim::ContourPointData &data) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual fmatvec::SqrMat computeOrientation(const fmatvec::Vec& q, const MBSim::ContourPointData &data) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual fmatvec::Vec computeVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual fmatvec::Vec computeAngularVelocity(const fmatvec::Vec& q, const fmatvec::Vec& u, const MBSim::ContourPointData &data) {
        throw MBSim::MBSimError("Not implemented");
      }
      virtual fmatvec::Mat computeJacobianOfMotion(const fmatvec::Vec& q, const MBSim::ContourPointData &data) {
        throw MBSim::MBSimError("Not implemented");
      }

      virtual void setDofDir(fmatvec::VecInt dofDirElement_) {
        dofDirs = dofDirElement_;
      }

      virtual fmatvec::VecInt getDofDirs() {
        return dofDirs;
      }

      /*!
       * \brief get the number of possible degrees of freedom (some may be locked, but this is not covered in the number)
       */
      virtual int getAddDoFSizeMax() const {
        return getNodesSize() * nodeDirs;
      }

      /*!
       * \brief get the number of Nodes
       */
      virtual int getNodesSize() const {
        return (order + 1);
      }

      FlexibleBody1SReferenceCurve * getParent() {
        return parent;
      }

      const std::vector<int> getFreeDofs() {
        return freeDoFs;
      }

      const fmatvec::Vec2 getAlpha() {
        return alpha;
      }

      int findLocalDof(int globalDof) {
        for (size_t i = 0; i < freeDoFs.size(); i++) {
          if (dofDirs(freeDoFs[i]) == globalDof)
            return freeDoFs[i];
        }
        return -1;
      }

      /*!
       * \brief compute the local position
       */
      virtual double computeXiLoc(double xiGlob);

      /*!
       * \brief compute the global position
       */
      virtual double computeXiGlob(double xiLoc);

      /*!
       * \brief compute the position and/or its derivative w.r.t. xi and/or theta
       */
      fmatvec::Vec3 computer(double xi, int derXi = 0, int derTheta = 0);

      /*!
       * \brief compute the velocity and/or its derivative w.r.t. xi and/or theta
       */
      fmatvec::Vec3 computev(double xi, int derXi = 0);

      /*!
       * \brief computes the derivative wrt the generalized position of the local element
       */
      fmatvec::Vec3 computedrdqk(double xi, int derXi, int qIndLoc);

      /*!
       * \brief compute the result of the ansatz function N of number j
       * Here hierarchical basis is used.
       * \param j     Number of DoF
       * \param xiLoc Local position on function
       * \param derXi Derivative w.r.t. Xi
       */
      virtual double computeN(int j, double xiLoc, int derXi);

      /*!
       * \brief compute Legendre Polynomial
       */
      virtual double computeLegendre(int j, double xiLoc, int derXi);

      /*!
       * \brief compute the column i of the B matrix (the column that is connected to the localDOF)
       * \remark here all columns can considered (also blocked DOFs)
       */
      fmatvec::Vec3 computeB(int dofDirLocal, double xiGlob, int derXi, int derTheta);

      /*!
       * \brief compute the multiplication of the B matrix with qf
       */
      fmatvec::Vec3 computeBqf(double xi, int derXi, int derTheta);

      /*!
       * \brief compute the multiplication of the B matrix with uf
       */
      fmatvec::Vec3 computeBuf(double xi, int derXi, int derTheta);

      /*!
       * \brief return the column of DOF with column i
       * \remark here all columns can considered (also blocked DOFs)
       */
      virtual fmatvec::Vec3 computeS(int columnEle, double xiGlob, int derXi);

      /*!
       * \brief compute S times the given vector
       */
      virtual fmatvec::Vec3 computeSTimeslocVec(double xiGlob, int derXi, const fmatvec::Vec & locVec);

      /*!
       * \brief compute A of the node times the respective shape functions times the given vector
       */
      virtual fmatvec::Vec3 computeAnodeTimesSTimeslocVec(double xiGlob, int derXi, int derTheta, const fmatvec::Vec & locVec);

      /*!
       * \brief compute the P matrix
       */
      fmatvec::Mat3xV computeP(double xi, int derXi);

      /*!
       * \brief compute the P matrix derived wrt the generalized position
       */
      fmatvec::Mat3xV computedPdqk(double xi, int dofDirLocal);

      /*!
       * \brief compute the local elongation
       */
      double computeeps(double xiGlob);

      /*!
       * \brief compute the derivative of the local elongation w.r.t. local Dof direction
       */
      double computedepsdqk(int localDofDir, double xiGlob);

      /*!
       * \brief compute the bending in t/n-plane
       */
      double computeKappan(double xiGlob);

      /*!
       * \brief compute the bending in t/b-plane
       */
      double computeKappab(double xiGlob);

      /*!
       * \brief compute derivative of kappan w.r.t. the given DoF
       */
      double computedKappandqk(double xiGlob, int qIndLocal);

      /*!
       * \brief compute derivative of kappab w.r.t. the given DoF
       */
      double computedKappabdqk(double xiGlob, int qIndLocal);

      /*!
       * \brief compute the change of bending energy in the t/n-plane at the position w.r.t the given DoF
       */
      double dWndqk(double xiGlob, int qInd);

      /*!
       * \brief update the borders as
       */
      void updateIntegrationBorders(double t);

    private:
      /*!
       * \brief parent body
       */
      FlexibleBody1SReferenceCurve * parent;

      /*!
       * \brief number of the element
       */
      int element;

      /*!
       * \brief vector that connects local DOFs of an element to the global DOFs
       * \remark negative values mean that those are blocked DoFs and therefore not free
       */
      fmatvec::VecInt dofDirs;

      /*
       * \brief start and end node in the global system
       */
      fmatvec::Vec2 alpha;

      /*!
       * \brief borders for the integration
       */
      std::vector<double> borders;

      /*!
       * \brief degree of the element
       */
      int order;

      /*!
       * \brief Degrees of freedom for one node (number of directions)
       */
      int nodeDirs;

      /*!
       * \brief number of free DoFs in the element
       */
      std::vector<int> freeDoFs;

      /*!
       * \brief list of polynomial coefficients
       * First index is the node
       * Second index is the derivative
       * third index is the actual coefficient
       */
      std::vector<std::vector<std::vector<double> > > coeffs;

      /*!
       * \brief function that holds the coefficient for the lagrange polynoms
       */
      std::vector<double> getLagrangeCoeffs(int nodeNo);

      /*!
       * \brief function that holds the coefficient for the hermite polynoms
       */
      std::vector<double> getHermiteCoeffs(int nodeNo);

      /*!
       * \brief Mass matrix of the element
       */
      fmatvec::SymMat M;

      /*!
       * \brief the right hand side vector
       */
      fmatvec::Vec h;

      /*!
       * \brief first part of h-vector coming from the kinetic energy
       */
      fmatvec::VecV hT1;

      /*!
       * \brief second part of h-vector coming from the kinetic energy
       */
      fmatvec::VecV hT2;

      /*!
       * \brief second part of h-vector coming from the kinetic energy
       */
      fmatvec::VecV hT3;

      /*!
       * \brief first part of h-vector coming from the potential energy (elongation)
       */
      fmatvec::VecV hV1;

      /*!
       * \brief second part of h-vector coming from the potential energy (normal bending)
       */
      fmatvec::VecV hV2;

      /*!
       * \brief third part of the h-vector coming from the potential energy (binormal bending)
       */
      fmatvec::VecV hV3;

      /*!
       * \brief fourth part of the h-vector coming from the potential energy (torsion)
       */
      fmatvec::VecV hV4;

      /*!
       * \brief computes the integral needed for the mass matrix (\int P^T P)
       */
      fmatvec::SymMatV integratePTP();

      /*!
       * \brief computes the integral for the first part of the h vector coming from the kinetic energy (\int P^T * P')
       */
      fmatvec::SqrMatV integratePTdPdxi();

      /*!
       * \brief computes the integral for the second part of the h vector coming from the kinetic energy (\int P^T * dPdt)
       */
      fmatvec::SqrMatV integratePTdPdt();

      /*!
       * \brief computes the integral for the third part of the h vector  coming from the kinetic energy (\int P^T * dPdqk)
       */
      fmatvec::SqrMatV integratePTdPdqk(int qInd);

      /*!
       * \brief computes the integral for the first part of the h vector  coming from the potential energy (intForWgamma)
       */
      double integrateForWgamma(int qInd);

      /*!
       * \brief computes the integral of the h vector coming from the potential energy (intForWn)
       */
      double integrateForWn(int qIndLoc);

      /*!
       * \brief computes the integral for the second part of the h vector coming from the potential energy (intForWb)
       */
      double integrateForWb(int qIndLoc);

      /*
       * \brief compute the global xi value of the given local dof direction
       * \remark the input values [0,1] have to global xi-values!
       */
      double computeXiOfDOF(int dofDirLocal);

      /*!
       * \brief switch between different versions of the A-Matrix
       */
      int AVers = 2;
  };

}

#endif /* FINITE_ELEMENT_1S_REFERENCE_CURVE_H_ */

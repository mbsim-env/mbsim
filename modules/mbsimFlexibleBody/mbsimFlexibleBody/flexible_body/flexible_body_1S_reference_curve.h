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

#ifndef FLEXIBLE_BODY_1S_REFERENCE_CURVE_H_
#define FLEXIBLE_BODY_1S_REFERENCE_CURVE_H_

#include <mbsimFlexibleBody/contours/neutral_contour/contour_1s_neutral_reference_curve.h>

#include <mbsimFlexibleBody/flexible_body.h>

#include <mbsim/functions/function.h>

namespace MBSimFlexibleBody {

  /*!
   * \brief this is a interface definition for possible implementations of the reference curve used in the FlexibleBody1SReferenceCurve
   */
  class ReferenceCurve {
    public:
      ReferenceCurve();
      virtual ~ReferenceCurve();

      /*!
       * \brief get the reference curve at a certain geometric ratio
       */
      virtual void computeReference() = 0;

      /*!
       * \brief computes the vector/point on the surface for specific derivatives
       */
      virtual fmatvec::Vec3 computeVecAt(double xi, double Theta, int derXi, int derTheta) = 0;

      /*!
       * \brief compute the only C1-continuous position in case the integral over the FE has to be split
       */
      virtual fmatvec::VecV computeC1Positions(const fmatvec::Vec & qRef) = 0;

    protected:
      /*!
       * \brief length of belt
       */
      double length;
  };

  /*!
   * \brief class that implements an one dimensional nonlinear beam that uses a reference curve for the nonlinearity together with overlaid deformations.
   *        Also the Eulerian coordinates ("S" not "s") are used to project the movement of the nodes back to their original position...
   */
  class FlexibleBody1SReferenceCurve : public FlexibleBodyContinuum<double> {
      friend class FlexibleBody1SReferenceCurveFE;
      friend class Contour1sNeutralFlexibleBody1SReferenceCurve;
      friend class funcForWgamma;
      friend class gethFunc;

    public:
      FlexibleBody1SReferenceCurve(const std::string & name, ReferenceCurve * refCurve);

      virtual ~FlexibleBody1SReferenceCurve();

      virtual void init(MBSim::Element::InitStage stage);
      virtual void initInfo(fmatvec::Vec q0 = fmatvec::Vec(0,fmatvec::NONINIT), fmatvec::Vec u0 = fmatvec::Vec(0,fmatvec::NONINIT));

      /*INHERITED INTERFACE*/
      virtual void BuildElements();
      virtual void updateh(double t, int k = 0);
      virtual void updateM(double t, int k = 0);
      virtual void updatedq(double t, double dt);
      virtual void updatedu(double t, double dt);
      virtual void updateud(double t, int i = 0);
      virtual void updateqd(double t);
      virtual void initz();
      virtual void plot(double t, double dt = 1.);

      virtual void GlobalVectorContribution(int, const fmatvec::Vec&, fmatvec::Vec&);

      virtual void GlobalMatrixContribution(int, const fmatvec::Mat3xV&, fmatvec::Mat3xV&);

      virtual void GlobalMatrixContribution(int, const fmatvec::Mat&, fmatvec::Mat&) {
        throw MBSim::MBSimError("NOT IMPLEMENTED: " + std::string(__func__));
      }
      virtual void GlobalMatrixContribution(int, const fmatvec::SymMat&, fmatvec::SymMat&);

      virtual void updateKinematicsForFrame(MBSim::ContourPointData&, MBSim::Frame::Feature, MBSim::Frame*);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData&, MBSim::Frame*) {
        throw MBSim::MBSimError("NOT IMPLEMENTED: " + std::string(__func__));
      }

      virtual void updateKinematicsAtNode(MBSimFlexibleBody::NodeFrame *frame, MBSim::Frame::Feature ff);

      virtual void setq0(fmatvec::Vec q0_) {
        FlexibleBodyContinuum<double>::setq0(q0_);
        qF << q0;
      }
      virtual void setu0(fmatvec::Vec u0_) {
        FlexibleBodyContinuum<double>::setu0(u0_);
        uF << u0;
      }

      /*GETTER and SETTER*/
      void setLength(double length_) {
        length = length_;
      }
      double getlength() const {
        return length;
      }
      void setDensity(double rho_) {
        rho = rho_;
      }
      double getrho() const {
        return rho;
      }
      void setCrossSectionalArea(double A_) {
        A = A_;
      }
      double getA() const {
        return A;
      }
      void setEModul(double E_) {
        E = E_;
      }
      double getE() const {
        return E;
      }
      void setNu(double nu_) {
        nu = nu_;
      }
      double getNu() const {
        return nu;
      }
      void setIn(double In_) {
        In = In_;
      }
      double getIn() const {
        return In;
      }
      void setIb(double Ib_) {
        Ib = Ib_;
      }
      double getIb() const {
        return Ib;
      }
      void setIt(double It_) {
        It = It_;
      }
      double getIt() const {
        return It;
      }
      void setdPulley(double dPulley_) {
        dPulley = dPulley_;
      }
      double getdPulley() const {
        return dPulley;
      }

      void setNumberElements(int elements_) {
        elements = elements_;
      }

      void setElementOrder(int elementOrder_) {
        elementOrder = elementOrder_;
      }

      void setNodeDofs(int nodeDofs_) {
        nodeDoFs = nodeDofs_;
      }

      void setUseSpatialReferenceKinematics(bool switch_) {
        useSpatialReferenceKinematics = switch_;
      }

      void setThetaDamping(double dTheta_) {
        dTheta = dTheta_;
      }

      void setLockedDofs(std::set<int> lockedDofs_) {
        lockedDofsFull = lockedDofs_;
      }

      void setLambdaqSwitches(const fmatvec::VecV & lambdaqSwitches_) {
        lambdaqFSwitches = lambdaqSwitches_;
      }

      void setLambdauSwitches(const fmatvec::VecV & lambdauSwitches_) {
        lambdauFSwitches = lambdauSwitches_;
      }

      virtual void setelongationActive(bool val) {
        elongationActive = val;
      }

      virtual void setnormalBendingActive(bool val) {
        normalBendingActive = val;
      }

      virtual double gets() {
        if (not usePT1)
          return q(0);
        else
          return qF(0);
      }

      virtual void updateStateDependentVariables(double t);

      /*!
       * \brief create a neutral phase as a basis for overlaid contours
       */
      Contour1sNeutralFlexibleBody1SReferenceCurve* createNeutralPhase(const std::string & contourName);

      /*!
       * \brief compute the physical strain at the Eulerian coordinate xi
       */
      double computePhysicalStrain(double xi);

      /*!
       * \brief compute the neutral state, i.e. h = 0
       */
      fmatvec::Vec computeNeutralState(const fmatvec::Vec & q0);

    private:
      /*!
       * \brief the reference curve
       */
      ReferenceCurve * refCurve;

      /*!
       * \brief number of elements used
       */
      int elements;

      /*!
       * \brief DoFs per element
       */
      int nodeDoFs;

      /*!
       * \brief DoFs per element
       */
      int elementOrder;

      /*!
       * \brief switch to only use kinematical reference deformation
       */
      bool useSpatialReferenceKinematics;

      /*!
       * \brief length of ring
       */
      double length;

      /*!
       * \brief density of the body
       */
      double rho;

      /*!
       * \brief cross section Area of the body
       */
      double A;

      /*!
       * \brief Youngs modulus of the body
       */
      double E;

      /*!
       * \brief Poisson-ratio of the body
       */
      double nu;

      /**
       * \brief area moment of inertia around normal axis
       */
      double In;

      /**
       * \brief area moment of inertia around binormal axis
       */
      double Ib;

      /**
       * \brief area moment of inertia around torsional axis
       */
      double It;

      /*!
       * \brief distance between pulleys
       */
      double dPulley;

      /*!
       * \brief Binormal of the curve
       */
      fmatvec::Vec3 b;

      /*!
       * \brief damping coefficient for theta
       */
      double dTheta;

      /*!
       * \brief this vector saves the locked DOFs of the system
       */
      std::set<int> lockedDofsFull;

      /*!
       * \brief vector for PT1 in positions
       */
      fmatvec::VecV qF;

      /*!
       * \brief vector for PT1 in velocities
       */
      fmatvec::VecV uF;

      /*!
       * \brief these are the switches to the the lambdaqSwitches
       * The first entrie is always the entro for "s" the second one always for "theta".
       * Then it depends on the element order but it actually follows just the global q-vector.
       * Thus for elementOrder 3 and planar it is like this:
       * lambdaqSwitches = [s, theta, qt, qn, qt', qn']
       * Thus for elementOrder 5 and spatial it is like this:
       * lambdaqSwitches = [s, theta, qt, qn, qb, qt', qn', qb'', qt'', qn'', qb'']
       * As here only the switches are set no one cars for the locked directions...
       */
      fmatvec::VecV lambdaqFSwitches;

      /*!
       * \brief these are the switches to the the lambdauSwitches
       * See the explanations for the qSwitches, however for the generalized velocities
       */
      fmatvec::VecV lambdauFSwitches;

      /*!
       * \brief lambda factors for qFs
       */
      fmatvec::VecV lambdaqF;

      /*!
       * \brief lambda factors for uFs
       */
      fmatvec::VecV lambdauF;

      /*!
       * \brief switch to use either the lambda PT1 version or not
       */
      bool usePT1 = false;

      /*!
       * \brief vector storing the values of the locked and not locked Dofs
       */
      std::vector<fmatvec::Vec> qElementAll;

      /*!
       * \brief vector storing the values of the locked and not locked Dofs generelaized velocities
       */
      std::vector<fmatvec::Vec> uElementAll;

      /*!
       * \brief number of how often the reference movement should be updated compared to the local movements
       */
      int updateReferenceEvery;

      /*!
       * \brief counter of how many updates have been done since the last time
       */
      int referenceNotUpdated;

      /*!
       * \brief marker since when the last update has been done on the reference
       */
      double tLastRefUpdate;

      /*!
       * \brief switch to enable / disable the elongation energy in the h-vector
       */
      bool elongationActive = true;

      /*!
       * \brief switch to enable / disable the normal bending energy in the h-vector
       */
      bool normalBendingActive = true;

      /*!
       * \brief computes the shear modulus G
       */
      double computeG() const {
        return E / (2 * (1 + nu));
      }

      /*!
       * \brief compute the position and/or its derivative w.r.t. xi and/or theta
       */
      fmatvec::Vec3 computer(double xi, int derXi = 0, int derTheta = 0);

      /*!
       * \brief computes the derivative wrt the generalized positions
       */
      fmatvec::Vec3 computedrdqk(double xi, int derXi, int qInd);

      /*!
       * \brief compute the velocity and/or its derivative w.r.t. xi and/or theta
       */
      fmatvec::Vec3 computev(double xi);

      /*!
       * \brief compute the position and its derivatives w.r.t. xi or theta at position xi and ratio Theta
       */
      fmatvec::Vec3 computerRef(double xi, int derXi = 0, int derTheta = 0);

      /*!
       * \brief compute the local transformation Matrix A and its derivatives w.r.t. xi or theta at position xi and ratio Theta
       */
      fmatvec::SqrMat3 computeARef(double xi, int derXi = 0, int derTheta = 0);

      /*!
       * \brief compute the normal vector at the given position
       */
      fmatvec::Vec3 computenRef(double xi, int derXi = 0, int derTheta = 0);

      /*!
       * \brief compute the column of the elements S matrix
       */
      fmatvec::Vec3 computeSElement(int globalDOF, double xi, int derXi);

      /*!
       * \brief compute the deformation vector, i.e. S*qf, for the ring
       */
      fmatvec::Vec3 computeSqfElement(double xi, int derXi);

      /*!
       * \brief compute the deformation vector, i.e. S*uf, for the ring
       */
      fmatvec::Vec3 computeSufElement(double xi, int derXi);

      /*!
       * \brief compute the P matrix
       */
      fmatvec::Mat3xV computeP(double xi, int derXi = 0);

      /*!
       * \brief compute the P matrix derived wrt the generalized position
       */
      fmatvec::Mat3xV computedPdqk(double xi, int qInd);

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
      fmatvec::Vec3 integrateForWgammaWnWtau(int qInd);

      /*!
       * \brief computes the integral for the second part of the h vector coming from the potential energy (intForWgamma)
       */
      double integrateForWb(int qInd);

      /*!
       * \brief find the element number of the given global coordinate
       * \remark: If xi is not valid (not big or too small it is changed!)
       */
      int findElement(double & xi);

      /*!
       * \brief find the elemental DOF Number out of the global DOF Number
       * \remark the first two local DOFs are locked
       */
      std::vector<std::pair<int, int> > getElementAndLocalDoFNo(int globalColumn);

      /*!
       * \brief get the number of DOFs for one element
       */
      int getEleDofs() const;
  };

  class gethFunc : public MBSim::Function<fmatvec::Vec(fmatvec::Vec)> {
    public:
      gethFunc(FlexibleBody1SReferenceCurve * refBody) :
          refBody(refBody) {
      }

      ~gethFunc() {
      }

      fmatvec::Vec operator()(const fmatvec::Vec & q) {
        refBody->setq(q);
        refBody->updateh(0, 0);
        return refBody->geth(0);
      }
    private:
      FlexibleBody1SReferenceCurve * refBody;
  };

}

#endif /* FLEXIBLE_BODY_1S_REFERENCE_CURVE_H_ */

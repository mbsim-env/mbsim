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

#ifndef _FLEXIBLE_BODY_1S_33_RCM_H_
#define _FLEXIBLE_BODY_1S_33_RCM_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/pointer.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_rcm.h"
#include "mbsimFlexibleBody/utils/revcardan.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/frame.h"
#include <mbsim/environment.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

#ifdef HAVE_NURBS
#define MY_PACKAGE_BUGREPORT PACKAGE_BUGREPORT
#define MY_PACKAGE_NAME PACKAGE_NAME
#define MY_PACKAGE_VERSION PACKAGE_VERSION
#define MY_PACKAGE_TARNAME PACKAGE_TARNAME
#define MY_PACKAGE_STRING PACKAGE_STRING
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "nurbs++/nurbs.h"
#undef PACKAGE_BUGREPORT
#undef PACKAGE_NAME
#undef PACKAGE_VERSION
#undef PACKAGE_TARNAME
#undef PACKAGE_STRING
#include "nurbs++/vector.h"
#define PACKAGE_BUGREPORT MY_PACKAGE_BUGREPORT
#define PACKAGE_NAME MY_PACKAGE_NAME
#define PACKAGE_VERSION MY_PACKAGE_VERSION
#define PACKAGE_TARNAME MY_PACKAGE_TARNAME
#define PACKAGE_STRING MY_PACKAGE_STRING
#endif


namespace MBSimFlexibleBody {

  /**
   * \brief spatial beam using Redundant Coordinate Method (RCM)
   * \author Thorsten Schindler
   * \date 2009-04-17 initial commit kernel_dev (Thorsten Schindler)
   * \date 2009-05-08 visualisation (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-23 implicit integration (Thorsten Schindler)
   * \date 2012-01-10 import / export position and velocity (Cebulla / Grundl)
   * \date 2012-05-14 added Contour1sFlexible for perlchain example (Thomas Cebulla)
   * \todo gyroscopic accelerations TODO
   * \todo inverse kinetics TODO
   */
  class FlexibleBody1s33RCM : public FlexibleBodyContinuum<fmatvec::Fixed<16>, double> {
    public:
      /**
       * \brief constructor
       * \param name of body
       * \param bool to specify open (cantilever) or closed (ring) structure
       */
      FlexibleBody1s33RCM(const std::string &name,bool openStructure);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody1s33RCM() {}

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat16& locMat, fmatvec::SymMat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SqrMat16& locMat, fmatvec::Mat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      virtual void exportPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ), const int & deg = 3, const bool & writePsFile = false);
      virtual void importPositionVelocity(const std::string & filenamePos, const std::string & filenameVel = std::string( ));
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s33RCM"; }
      void initializeUsingXML(TiXmlElement * element);
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberElements(int n);
      int getNumberElements(){return Elements;}
      void setLength(double L_);
      double getLength(){return L;}
      void setEGModuls(double E_,double G_);
      void setDensity(double rho_);
      void setCrossSectionalArea(double A_);
      void setMomentsInertia(double I1_,double I2_,double I0_);
      void setCylinder(double cylinderRadius_);
      void setMaterialDamping(double epstD_,double k0D_);

      void setGauss(int nGauss);
      void setCuboid(double cuboidBreadth_,double cuboidHeight_);
      void setCurlRadius(double R1_,double R2_);
      void setLehrDamping(double epstL_,double k0L_);
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif
      /***************************************************/

      /**
       * \brief compute state (positions, angles, velocities, differentiated angles) at Lagrangian coordinate in local FE coordinates
       * \param Lagrangian coordinate
       */
      fmatvec::Vec12 computeState(double x);

      /**
       * initialise beam only for giving information with respect to state, number elements, length, (not for simulation)
       */
      void initInfo();

    private:
      /**
       * \brief contours
       */
      CylinderFlexible<fmatvec::Fixed<16> > *cylinder;
      FlexibleBand<fmatvec::Fixed<16> > *top, *bottom, *left, *right;
      Contour1sFlexible<fmatvec::Fixed<16> > *neutralFibre;

      /**
       * \brief angle parametrisation
       */
      RevCardanPtr angle;

      /**
       * \brief number of elements
       */
      int Elements;

      /**
       * \brief length of entire beam and finite elements
       */
      double L, l0;

      /**
       * \brief elastic modules
       */
      double E, G;

      /**
       * \brief area of cross-section
       */
      double A;

      /**
       * \brief area moment of inertia
       */
      double I1, I2, I0;

      /**
       * \brief density
       */
      double rho;

      /**
       * \brief radius of undeformed shape
       */
      double R1, R2;

      /**
       * \brief damping
       */
      double epstD, k0D, epstL, k0L;

      /**
       * \brief open or closed structure
       */
      bool openStructure;

      /**
       * \brief initialised FLAG
       */
      bool initialised;

      /**
       * \brief number of Gauss points for rotational kinetic energy
       */
      int nGauss;

      /**
       * \brief contour data
       */
      double cylinderRadius, cuboidBreadth, cuboidHeight;

      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);
  };

  inline void FlexibleBody1s33RCM::setGauss(int nGauss_) { nGauss = nGauss_; }
  inline void FlexibleBody1s33RCM::setCylinder(double cylinderRadius_) { cylinderRadius = cylinderRadius_; }
  inline void FlexibleBody1s33RCM::setCuboid(double cuboidBreadth_,double cuboidHeight_) { cuboidBreadth = cuboidBreadth_; cuboidHeight = cuboidHeight_; }
  inline void FlexibleBody1s33RCM::setLength(double L_) { L = L_; }
  inline void FlexibleBody1s33RCM::setEGModuls(double E_,double G_) { E = E_; G = G_; }
  inline void FlexibleBody1s33RCM::setCrossSectionalArea(double A_) { A = A_; }
  inline void FlexibleBody1s33RCM::setMomentsInertia(double I1_,double I2_,double I0_) { I1 = I1_; I2 = I2_; I0 = I0_; }
  inline void FlexibleBody1s33RCM::setDensity(double rho_) { rho = rho_;}
  inline void FlexibleBody1s33RCM::setCurlRadius(double R1_,double R2_) { R1 = R1_; R2 = R2_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1,R2); }
  inline void FlexibleBody1s33RCM::setMaterialDamping(double epstD_,double k0D_) {epstD = epstD_; k0D = k0D_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements*epstD,Elements*k0D); }
  inline void FlexibleBody1s33RCM::setLehrDamping(double epstL_,double k0L_) { epstL = epstL_; k0L = k0L_; if(initialised) for(int i=0;i<Elements;i++) static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements*epstL,Elements*k0L); }

  inline FlexibleBody1s33RCM::FlexibleBody1s33RCM(const std::string &name, bool openStructure_) :
      FlexibleBodyContinuum<fmatvec::Fixed<16>, double>(name), cylinder(new CylinderFlexible<fmatvec::Fixed<16> >("Cylinder")), top(new FlexibleBand<fmatvec::Fixed<16> >("Top")), bottom(new FlexibleBand<fmatvec::Fixed<16> >("Bottom")), left(new FlexibleBand<fmatvec::Fixed<16> >("Left")), right(new FlexibleBand<fmatvec::Fixed<16> >("Right")), neutralFibre(new Contour1sFlexible<fmatvec::Fixed<16> >("NeutralFibre")), angle(new RevCardan()), Elements(0), L(0.), l0(0.), E(0.), G(0.), A(0.), I1(0.), I2(0.), I0(0.), rho(0.), R1(0.), R2(0.), epstD(0.), k0D(0.), epstL(0.), k0L(0.), openStructure(openStructure_), initialised(false), nGauss(3), cylinderRadius(0.), cuboidBreadth(0.), cuboidHeight(0.) {
    Body::addContour(cylinder);
    Body::addContour(top);
    Body::addContour(bottom);
    Body::addContour(left);
    Body::addContour(right);
    Body::addContour(neutralFibre);
  }

  inline void FlexibleBody1s33RCM::BuildElements() {
    for (int i = 0; i < Elements; i++) {
      int j = 10 * i; // start index in entire beam coordinates

      if (i < Elements - 1 || openStructure) {
        qElement[i] = q(j, j + 15);
        uElement[i] = u(j, j + 15);
      }
      else { // last FE-Beam for closed structure
        qElement[i].set(fmatvec::Index(0, 9), q(j, j + 9));
        uElement[i].set(fmatvec::Index(0, 9), u(j, j + 9));
        qElement[i].set(fmatvec::Index(10, 15), q(0, 5));
        if (q(j + 5) < q(5))
          qElement[i](15) -= 2. * M_PI;
        else
          qElement[i](15) += 2. * M_PI;
        uElement[i].set(fmatvec::Index(10, 15), u(0, 5));
      }
    }
  }

  inline void FlexibleBody1s33RCM::GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloVec(j, j + 15) += locVec;
    }
    else { // last FE for closed structure
      gloVec(j, j + 9) += locVec(0, 9);
      gloVec(0, 5) += locVec(10, 15);
    }
  }

  inline void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(fmatvec::Index(j, j + 15), fmatvec::Index(j, j + 15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(fmatvec::Index(j, j + 9), fmatvec::Index(j, j + 9)) += locMat(fmatvec::Index(0, 9), fmatvec::Index(0, 9));
      gloMat(fmatvec::Index(j, j + 9), fmatvec::Index(0, 5)) += locMat(fmatvec::Index(0, 9), fmatvec::Index(10, 15));
      gloMat(fmatvec::Index(0, 5), fmatvec::Index(j, j + 9)) += locMat(fmatvec::Index(10, 15), fmatvec::Index(0, 9));
      gloMat(fmatvec::Index(0, 5), fmatvec::Index(0, 5)) += locMat(fmatvec::Index(10, 15), fmatvec::Index(10, 15));
    }
  }

  inline void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const fmatvec::SymMat16& locMat, fmatvec::SymMat& gloMat) {
    int j = 10 * n; // start index in entire beam coordinates

    if (n < Elements - 1 || openStructure) {
      gloMat(fmatvec::Index(j, j + 15)) += locMat;
    }
    else { // last FE for closed structure
      gloMat(fmatvec::Index(j, j + 9)) += locMat(fmatvec::Range0x9());
      gloMat(fmatvec::Index(j, j + 9), fmatvec::Index(0, 5)) += locMat(fmatvec::Range0x9(), fmatvec::Range10x15());
      gloMat(fmatvec::Index(0, 5)) += locMat(fmatvec::Range10x15());
    }
  }

  inline void FlexibleBody1s33RCM::GlobalMatrixContribution(int n, const fmatvec::SqrMat16& locMat, fmatvec::Mat& gloMat) {
    throw MBSim::MBSimError("ERROR: FlexibleBody1s33RCM::GlobalMatrixContribution: Not implemented!");
  }

  inline void FlexibleBody1s33RCM::updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame) {
    if (cp.getContourParameterType() == MBSim::CONTINUUM) { // frame on continuum
      fmatvec::Vec12 X = computeState(cp.getLagrangeParameterPosition()(0)); // state of affected FE
      const fmatvec::Vec3 &Phi = X(fmatvec::Range3x5());
      const fmatvec::Vec3 &Phit = X(fmatvec::Range9x11());

      if (ff == MBSim::position || ff == MBSim::position_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * X(fmatvec::Range0x2()));
      if (ff == MBSim::firstTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(1, frameOfReference->getOrientation() * angle->computet(Phi)); // tangent
      if (ff == MBSim::normal || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(0, frameOfReference->getOrientation() * angle->computen(Phi)); // normal
      if (ff == MBSim::secondTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(2, crossProduct(cp.getFrameOfReference().getOrientation().col(0), cp.getFrameOfReference().getOrientation().col(1))); // binormal
      if (ff == MBSim::velocity || ff == MBSim::velocity_cosy || ff == MBSim::velocities || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * X(fmatvec::Range6x8()));
      if (ff == MBSim::angularVelocity || ff == MBSim::velocities || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * angle->computeOmega(Phi, Phit));
    }
    else if (cp.getContourParameterType() == MBSim::NODE) { // frame on node
      int node = cp.getNodeNumber();
      const fmatvec::Vec3 &Phi = q(10 * node + 3, 10 * node + 5);
      const fmatvec::Vec3 &Phit = u(10 * node + 3, 10 * node + 5);

      if (ff == MBSim::position || ff == MBSim::position_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setPosition(frameOfReference->getPosition() + frameOfReference->getOrientation() * q(10 * node + 0, 10 * node + 2));
      if (ff == MBSim::firstTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(1, frameOfReference->getOrientation() * angle->computet(Phi)); // tangent
      if (ff == MBSim::normal || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(0, frameOfReference->getOrientation() * angle->computen(Phi)); // normal
      if (ff == MBSim::secondTangent || ff == MBSim::cosy || ff == MBSim::position_cosy || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().getOrientation().set(2, crossProduct(cp.getFrameOfReference().getOrientation().col(0), cp.getFrameOfReference().getOrientation().col(1))); // binormal (cartesian system)
      if (ff == MBSim::velocity || ff == MBSim::velocities || ff == MBSim::velocity_cosy || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setVelocity(frameOfReference->getOrientation() * u(10 * node + 0, 10 * node + 2));
      if (ff == MBSim::angularVelocity || ff == MBSim::velocities || ff == MBSim::velocities_cosy || ff == MBSim::all)
        cp.getFrameOfReference().setAngularVelocity(frameOfReference->getOrientation() * angle->computeOmega(Phi, Phit));
    }
    else
      throw MBSim::MBSimError("ERROR(FlexibleBody1s33RCM::updateKinematicsForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    if (frame != 0) { // frame should be linked to contour point data
      frame->setPosition(cp.getFrameOfReference().getPosition());
      frame->setOrientation(cp.getFrameOfReference().getOrientation());
      frame->setVelocity(cp.getFrameOfReference().getVelocity());
      frame->setAngularVelocity(cp.getFrameOfReference().getAngularVelocity());
    }
  }

  inline void FlexibleBody1s33RCM::updateJacobiansForFrame(MBSim::ContourPointData &cp, MBSim::Frame *frame) {
    fmatvec::Index All(0, 5);
    fmatvec::Index One(0, 2);
    fmatvec::Mat Jacobian(qSize, 6, fmatvec::INIT, 0.);

    if (cp.getContourParameterType() == MBSim::CONTINUUM) { // frame on continuum
      double sLocal;
      int currentElement;
      BuildElement(cp.getLagrangeParameterPosition()(0), sLocal, currentElement); // compute parameters of affected FE
      fmatvec::Mat Jtmp = static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computeJacobianOfMotion(qElement[currentElement], sLocal); // this local ansatz yields continuous and finite wave propagation

      if (currentElement < Elements - 1 || openStructure) {
        Jacobian(fmatvec::Index(10 * currentElement, 10 * currentElement + 15), All) = Jtmp;
      }
      else { // last FE for closed structure
        Jacobian(fmatvec::Index(10 * currentElement, 10 * currentElement + 9), All) = Jtmp(fmatvec::Index(0, 9), All);
        Jacobian(fmatvec::Index(0, 5), All) = Jtmp(fmatvec::Index(10, 15), All);
      }
    }
    else if (cp.getContourParameterType() == MBSim::NODE) { // frame on node
      int node = cp.getNodeNumber();

      fmatvec::Vec p = q(10 * node + 3, 10 * node + 5);
      fmatvec::Vec t = angle->computet(p);
      fmatvec::Vec n = angle->computen(p);
      fmatvec::Vec b = angle->computeb(p);
      fmatvec::SqrMat3 tp = angle->computetq(p);
      fmatvec::SqrMat3 np = angle->computenq(p);
      fmatvec::SqrMat3 bp = angle->computebq(p);

      // translation
      Jacobian(fmatvec::Index(10 * node, 10 * node + 2), One) << fmatvec::SqrMat(3, fmatvec::EYE);

      // rotation
      Jacobian(fmatvec::Index(10 * node + 3, 10 * node + 5), 3) = t(1) * tp(fmatvec::Index(2,2), fmatvec::Index(0,2)).T() + n(1) * np(fmatvec::Index(2,2), fmatvec::Index(0,2)).T() + b(1) * bp(fmatvec::Index(2,2), fmatvec::Index(0,2)).T();
      Jacobian(fmatvec::Index(10 * node + 3, 10 * node + 5), 4) = t(2) * tp(fmatvec::Index(0,0), fmatvec::Index(0,2)).T() + n(2) * np(fmatvec::Index(0,0), fmatvec::Index(0,2)).T() + b(2) * bp(fmatvec::Index(0,0), fmatvec::Index(0,2)).T();
      Jacobian(fmatvec::Index(10 * node + 3, 10 * node + 5), 5) = t(0) * tp(fmatvec::Index(1,1), fmatvec::Index(0,2)).T() + n(0) * np(fmatvec::Index(1,1), fmatvec::Index(0,2)).T() + b(0) * bp(fmatvec::Index(1,1), fmatvec::Index(0,2)).T();
    }
    else
      throw MBSim::MBSimError("ERROR(FlexibleBody1s33RCM::updateJacobiansForFrame): ContourPointDataType should be 'NODE' or 'CONTINUUM'");

    cp.getFrameOfReference().setJacobianOfTranslation(frameOfReference->getOrientation() * Jacobian(0, 0, qSize - 1, 2).T());
    cp.getFrameOfReference().setJacobianOfRotation(frameOfReference->getOrientation() * Jacobian(0, 3, qSize - 1, 5).T());
    // cp.getFrameOfReference().setGyroscopicAccelerationOfTranslation(TODO)
    // cp.getFrameOfReference().setGyroscopicAccelerationOfRotation(TODO)

    if (frame != 0) { // frame should be linked to contour point data
      frame->setJacobianOfTranslation(cp.getFrameOfReference().getJacobianOfTranslation());
      frame->setJacobianOfRotation(cp.getFrameOfReference().getJacobianOfRotation());
      frame->setGyroscopicAccelerationOfTranslation(cp.getFrameOfReference().getGyroscopicAccelerationOfTranslation());
      frame->setGyroscopicAccelerationOfRotation(cp.getFrameOfReference().getGyroscopicAccelerationOfRotation());
    }
  }

  inline void FlexibleBody1s33RCM::init(MBSim::InitStage stage) {
    if (stage == MBSim::unknownStage) {
      FlexibleBodyContinuum<fmatvec::Fixed<16>, double>::init(stage);

      initialised = true;

      /* cylinder */
      cylinder->setAlphaStart(0.);
      cylinder->setAlphaEnd(L);

      if (userContourNodes.size() == 0) {
        fmatvec::Vec contourNodes(Elements + 1);
        for (int i = 0; i <= Elements; i++)
          contourNodes(i) = L / Elements * i; // own search area for each element
        cylinder->setNodes(contourNodes);
      }
      else {
        cylinder->setNodes(userContourNodes);
      }

      cylinder->setRadius(cylinderRadius);

      /* cuboid */
      top->setCn(fmatvec::Vec2("[1.;0.]"));
      bottom->setCn(fmatvec::Vec2("[-1.;0.]"));
      left->setCn(fmatvec::Vec2("[0.;-1.]"));
      right->setCn(fmatvec::Vec2("[0.;1.]"));

      top->setAlphaStart(0.);
      top->setAlphaEnd(L);

      bottom->setAlphaStart(0.);
      bottom->setAlphaEnd(L);

      left->setAlphaStart(0.);
      left->setAlphaEnd(L);

      right->setAlphaStart(0.);
      right->setAlphaEnd(L);

      /* neutral fibre  */
      neutralFibre->getFrame()->setOrientation(frameOfReference->getOrientation());
      neutralFibre->setAlphaStart(0.);
      neutralFibre->setAlphaEnd(L);

      if (userContourNodes.size() == 0) {
        fmatvec::Vec contourNodes(Elements + 1);
        for (int i = 0; i <= Elements; i++)
          contourNodes(i) = L / Elements * i;
        top->setNodes(contourNodes);
        bottom->setNodes(contourNodes);
        left->setNodes(contourNodes);
        right->setNodes(contourNodes);
        neutralFibre->setNodes(contourNodes);
      }
      else {
        top->setNodes(userContourNodes);
        bottom->setNodes(userContourNodes);
        left->setNodes(userContourNodes);
        right->setNodes(userContourNodes);
        neutralFibre->setNodes(userContourNodes);
      }

      top->setWidth(cuboidBreadth);
      bottom->setWidth(cuboidBreadth);
      top->setNormalDistance(0.5 * cuboidHeight);
      bottom->setNormalDistance(0.5 * cuboidHeight);
      left->setWidth(cuboidHeight);
      right->setWidth(cuboidHeight);
      left->setNormalDistance(0.5 * cuboidBreadth);
      right->setNormalDistance(0.5 * cuboidBreadth);

      l0 = L / Elements;
      fmatvec::Vec3 g = frameOfReference->getOrientation().T() * MBSim::MBSimEnvironment::getInstance()->getAccelerationOfGravity();

      for (int i = 0; i < Elements; i++) {
        discretization.push_back(new FiniteElement1s33RCM(l0, rho, A, E, G, I1, I2, I0, g, angle));
        qElement.push_back(fmatvec::Vec16());
        uElement.push_back(fmatvec::Vec16());
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setGauss(nGauss);
        if (fabs(R1) > MBSim::epsroot() || fabs(R2) > MBSim::epsroot())
          static_cast<FiniteElement1s33RCM*>(discretization[i])->setCurlRadius(R1, R2);
        static_cast<FiniteElement1s33RCM*>(discretization[i])->setMaterialDamping(Elements * epstD, Elements * k0D);
        if (fabs(epstD) < MBSim::epsroot())
          static_cast<FiniteElement1s33RCM*>(discretization[i])->setLehrDamping(Elements * epstL, Elements * k0L);
      }
    }
    else
      FlexibleBodyContinuum<fmatvec::Fixed<16>, double>::init(stage);
  }

  inline void FlexibleBody1s33RCM::plot(double t, double dt) {
    if (getPlotFeature(MBSim::plotRecursive) == MBSim::enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(MBSim::openMBV) == MBSim::enabled && openMBVBody) {
        std::vector<double> data;
        data.push_back(t);
        double ds = openStructure ? L / (((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints() - 1) : L / (((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints() - 2);
        for (int i = 0; i < ((OpenMBV::SpineExtrusion*) openMBVBody)->getNumberOfSpinePoints(); i++) {
          fmatvec::Vec12 X = computeState(ds * i);
          fmatvec::Vec3 pos = frameOfReference->getPosition() + frameOfReference->getOrientation() * X(fmatvec::Range0x2());
          data.push_back(pos(0)); // global x-position
          data.push_back(pos(1)); // global y-position
          data.push_back(pos(2)); // global z-position
          data.push_back(X(3)); // local twist
        }
        ((OpenMBV::SpineExtrusion*) openMBVBody)->append(data);
      }
#endif
    }
    FlexibleBodyContinuum<fmatvec::Fixed<16>, double>::plot(t, dt);
  }

  inline void FlexibleBody1s33RCM::setNumberElements(int n) {
    Elements = n;
    if (openStructure)
      qSize = 10 * n + 6;
    else
      qSize = 10 * n;

    fmatvec::Vec q0Tmp;
    if (q0.size())
      q0Tmp = q0.copy();
    q0.resize(qSize, fmatvec::INIT, 0.);
    if (q0Tmp.size()) {
      if (q0Tmp.size() == q0.size())
        q0 = q0Tmp.copy();
      else
        throw MBSim::MBSimError("Error in dimension of q0 of FlexibleBody1s33RCM \"" + name + "\"!");
    }

    uSize[0] = qSize;
    uSize[1] = qSize; // TODO
    fmatvec::Vec u0Tmp;
    if (u0.size())
      u0Tmp = u0.copy();
    u0.resize(uSize[0], fmatvec::INIT, 0.);
    if (u0Tmp.size()) {
      if (u0Tmp.size() == u0.size())
        u0 = u0Tmp.copy();
      else
        throw MBSim::MBSimError("Error in dimension of u0 of FlexibleBody1s33RCM \"" + name + "\"!");
    }
  }

  fmatvec::Vec12 FlexibleBody1s33RCM::computeState(double sGlobal) {
    double sLocal;
    int currentElement;
    BuildElement(sGlobal, sLocal, currentElement); // Lagrange parameter of affected FE
    return static_cast<FiniteElement1s33RCM*>(discretization[currentElement])->computeState(qElement[currentElement], uElement[currentElement], sLocal);
  }

  inline void FlexibleBody1s33RCM::initInfo() {
    FlexibleBodyContinuum<fmatvec::Fixed<16>, double>::init(MBSim::unknownStage);
    l0 = L / Elements;
    for (int i = 0; i < Elements; i++) {
      discretization.push_back(new FiniteElement1s33RCM(l0, rho, A, E, G, I1, I2, I0, fmatvec::Vec3(), angle));
      qElement.push_back(fmatvec::Vec16());
      uElement.push_back(fmatvec::Vec16());
    }
    BuildElements();
  }

  inline void FlexibleBody1s33RCM::BuildElement(const double& sGlobal, double& sLocal, int& currentElement) {
    double remainder = fmod(sGlobal, L);
    if (openStructure && sGlobal >= L)
      remainder += L; // remainder \in (-eps,L+eps)
    if (!openStructure && sGlobal < 0.)
      remainder += L; // remainder \in [0,L)

    currentElement = int(remainder / l0);
    sLocal = remainder - (0.5 + currentElement) * l0; // Lagrange-Parameter of the affected FE with sLocal==0 in the middle of the FE and sGlobal==0 at the beginning of the beam

    if (currentElement >= Elements && openStructure) { // contact solver computes to large sGlobal at the end of the entire beam is not considered only for open structure
      currentElement = Elements - 1;
      sLocal += l0;
    }
  }

  inline void FlexibleBody1s33RCM::initializeUsingXML(TiXmlElement * element) {
    FlexibleBody::initializeUsingXML(element);
    TiXmlElement * e;

    // frames
    e = element->FirstChildElement(MBSIMFLEXNS"frames")->FirstChildElement();
    while (e && e->ValueStr() == MBSIMFLEXNS"frameOnFlexibleBody1s") {
      TiXmlElement *ec = e->FirstChildElement();
      MBSim::Frame *f = new MBSim::Frame(ec->Attribute("name"));
      f->initializeUsingXML(ec);
      ec = ec->NextSiblingElement();
      addFrame(f, getDouble(ec));
      e = e->NextSiblingElement();
    }

    //other properties

    e = element->FirstChildElement(MBSIMFLEXNS"numberOfElements");
    setNumberElements(getInt(e));
    e = element->FirstChildElement(MBSIMFLEXNS"length");
    setLength(getDouble(e));

    e = element->FirstChildElement(MBSIMFLEXNS"youngsModulus");
    double E = getDouble(e);
    e = element->FirstChildElement(MBSIMFLEXNS"shearModulus");
    double G = getDouble(e);
    setEGModuls(E, G);

    e = element->FirstChildElement(MBSIMFLEXNS"density");
    setDensity(getDouble(e));
    e = element->FirstChildElement(MBSIMFLEXNS"crossSectionArea");
    setCrossSectionalArea(getDouble(e));

    e = element->FirstChildElement(MBSIMFLEXNS"momentOfInertia");
    fmatvec::Vec TempVec2 = getVec(e);
    setMomentsInertia(TempVec2(0), TempVec2(1), TempVec2(2));

    e = element->FirstChildElement(MBSIMFLEXNS"radiusOfContourCylinder");
    setCylinder(getDouble(e));

    e = element->FirstChildElement(MBSIMFLEXNS"dampingOfMaterial");
    double thetaEps = getDouble(e->FirstChildElement(MBSIMFLEXNS"prolongational"));
    double thetaKappa0 = getDouble(e->FirstChildElement(MBSIMFLEXNS"torsional"));
    setMaterialDamping(thetaEps, thetaKappa0);

#ifdef HAVE_OPENMBVCPPINTERFACE
    e = element->FirstChildElement(MBSIMFLEXNS"openMBVBody");
    if (e) {
      OpenMBV::SpineExtrusion *rb = dynamic_cast<OpenMBV::SpineExtrusion*>(OpenMBV::ObjectFactory::createObject(e->FirstChildElement()));
      setOpenMBVSpineExtrusion(rb);
      rb->initializeUsingXML(e->FirstChildElement());
      rb->setNumberOfSpinePoints(4 * Elements + 1);
    }
#endif
  }

  inline void FlexibleBody1s33RCM::exportPositionVelocity(const std::string & filenamePos, const std::string & filenameVel /*= string( )*/, const int & deg /* = 3*/, const bool &writePsFile /*= false*/) {
#ifdef HAVE_NURBS

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;

    if (!openStructure) {
      PLib::Vector<PLib::HPoint3Dd> NodelistPos(Elements + deg);
      PLib::Vector<PLib::HPoint3Dd> NodelistVel(Elements + deg);

      for (int i = 0; i < Elements + deg; i++) {  // +deg-Elements are needed, as the curve is closed
        MBSim::ContourPointData cp(i);
        if (i >= Elements)
          cp.getNodeNumber() = i - Elements;

        updateKinematicsForFrame(cp, MBSim::position);
        NodelistPos[i] = PLib::HPoint3Dd(cp.getFrameOfReference().getPosition()(0), cp.getFrameOfReference().getPosition()(1), cp.getFrameOfReference().getPosition()(2), 1);

        if (not filenameVel.empty()) {
          updateKinematicsForFrame(cp, MBSim::velocity_cosy);

          fmatvec::SqrMat3 TMPMat = cp.getFrameOfReference().getOrientation();
          fmatvec::SqrMat3 AKI;
          AKI.set(0, trans(TMPMat.col(1)));
          AKI.set(1, trans(TMPMat.col(0)));
          AKI.set(2, trans(TMPMat.col(2)));
          fmatvec::Vec3 Vel = AKI * cp.getFrameOfReference().getVelocity();

          NodelistVel[i] = PLib::HPoint3Dd(Vel(0), Vel(1), Vel(2), 1);
        }
      }

      /*create own uVec and uvec like in nurbsdisk_2s*/
      PLib::Vector<double> uvec = PLib::Vector<double>(Elements + deg);
      PLib::Vector<double> uVec = PLib::Vector<double>(Elements + deg + deg + 1);

      const double stepU = L / Elements;

      uvec[0] = 0;
      for (int i = 1; i < uvec.size(); i++) {
        uvec[i] = uvec[i - 1] + stepU;
      }

      uVec[0] = (-deg) * stepU;
      for (int i = 1; i < uVec.size(); i++) {
        uVec[i] = uVec[i - 1] + stepU;
      }

      curvePos.globalInterpClosedH(NodelistPos, uvec, uVec, deg);
      curvePos.write(filenamePos.c_str());

      if (writePsFile) {
        std::string psfile = filenamePos + ".ps";

        std::cout << curvePos.writePS(psfile.c_str(), 0, 2.0, 5, false) << std::endl;
      }

      if (not filenameVel.empty()) {
        curveVel.globalInterpClosedH(NodelistVel, uvec, uVec, deg);
        curveVel.write(filenameVel.c_str());
      }
    }
#else
    throw MBSimError("No Nurbs-Library installed ...");
#endif
  }

  inline void FlexibleBody1s33RCM::importPositionVelocity(const std::string & filenamePos, const std::string & filenameVel /* = string( )*/) {
#ifdef HAVE_NURBS

    int DEBUGLEVEL = 0;

    PlNurbsCurved curvePos;
    PlNurbsCurved curveVel;
    curvePos.read(filenamePos.c_str());
    if (not filenameVel.empty())
      curveVel.read(filenameVel.c_str());

    l0 = L / Elements;
    fmatvec::Vec q0Dummy(q0.size(), fmatvec::INIT, 0.);
    fmatvec::Vec u0Dummy(u0.size(), fmatvec::INIT, 0.);
    PLib::Point3Dd prevBinStart;

    for (int i = 0; i < Elements; i++) {
      PLib::Point3Dd posStart = curvePos.pointAt(i * l0);
      PLib::Point3Dd pos1Quart = curvePos.pointAt(i * l0 + l0 / 4.);
      PLib::Point3Dd posHalf = curvePos.pointAt(i * l0 + l0 / 2.);
      PLib::Point3Dd pos3Quart = curvePos.pointAt(i * l0 + l0 * 3. / 4.);
      PLib::Point3Dd tangStart = curvePos.derive3D(i * l0, 1);
      tangStart /= norm(tangStart);
      PLib::Point3Dd tangHalf = curvePos.derive3D(i * l0 + l0 / 2., 1);
      PLib::Point3Dd binStart = curvePos.derive3D(i * l0, 2);
      binStart = crossProduct(binStart, tangStart);
      binStart /= norm(binStart);
      if (i > 0) {
        if (dot(prevBinStart, binStart) < 0)
          binStart = -1. * binStart;
      }
      prevBinStart = binStart;
      PLib::Point3Dd norStart = crossProduct(binStart, tangStart);

      q0Dummy(i * 10) = posStart.x(); // x
      q0Dummy(i * 10 + 1) = posStart.y(); // y
      q0Dummy(i * 10 + 2) = posStart.z(); // z

      fmatvec::SqrMat3 AIK;
      AIK(0, 0) = tangStart.x();
      AIK(1, 0) = tangStart.y();
      AIK(2, 0) = tangStart.z();
      AIK(0, 1) = norStart.x();
      AIK(1, 1) = norStart.y();
      AIK(2, 1) = norStart.z();
      AIK(0, 2) = binStart.x();
      AIK(1, 2) = binStart.y();
      AIK(2, 2) = binStart.z();
      fmatvec::Vec AlphaBetaGamma = MBSim::AIK2RevCardan(AIK);
      //q0Dummy(i*10+3) = AlphaBetaGamma(0); // alpha angle currently set to zero
      q0Dummy(i * 10 + 4) = AlphaBetaGamma(1);
      q0Dummy(i * 10 + 5) = AlphaBetaGamma(2);

      q0Dummy(i * 10 + 6) = -PLib::absolute((pos1Quart.y() - posHalf.y()) * (-tangHalf.z()) - (pos1Quart.z() - posHalf.z()) * (-tangHalf.y())) / sqrt(tangHalf.y() * tangHalf.y() + tangHalf.z() * tangHalf.z()); // cL1
      q0Dummy(i * 10 + 7) = -PLib::absolute((pos3Quart.y() - posHalf.y()) * tangHalf.z() - (pos3Quart.z() - posHalf.z()) * tangHalf.y()) / sqrt(tangHalf.y() * tangHalf.y() + tangHalf.z() * tangHalf.z()); // cR1
      q0Dummy(i * 10 + 8) = -PLib::absolute((pos1Quart.x() - posHalf.x()) * (-tangHalf.y()) - (pos1Quart.y() - posHalf.y()) * (-tangHalf.x())) / sqrt(tangHalf.x() * tangHalf.x() + tangHalf.y() * tangHalf.y()); // cL2
      q0Dummy(i * 10 + 9) = -PLib::absolute((pos3Quart.x() - posHalf.x()) * tangHalf.y() - (pos3Quart.y() - posHalf.y()) * tangHalf.x()) / sqrt(tangHalf.x() * tangHalf.x() + tangHalf.y() * tangHalf.y()); // cR2

      if (not filenameVel.empty()) {
        PLib::Point3Dd velStart = curveVel.pointAt(i * l0);

        fmatvec::Vec3 velK;
        velK(0) = velStart.x();
        velK(1) = velStart.y();
        velK(2) = velStart.z();
        fmatvec::Vec3 velI = trans(frameOfReference->getOrientation()) * AIK * velK;

        u0Dummy(i * 10) = velI(0);
        u0Dummy(i * 10 + 1) = velI(1);
        u0Dummy(i * 10 + 2) = velI(2);
      }

      if (DEBUGLEVEL == 1) {
        cout << "START(" << i + 1 << ",1:end) = [" << posStart << "];" << endl;
        cout << "Tangent(" << i + 1 << ",1:end) = [" << tangStart << "];" << endl;
        cout << "Normal(" << i + 1 << ",1:end) = [" << norStart << "];" << endl;
        cout << "Binormal(" << i + 1 << ",1:end) = [" << binStart << "];" << endl;

        cout << "alpha_Old(" << i + 1 << ") = " << q(i * 10 + 3) << ";" << endl;
        cout << "beta_Old(" << i + 1 << ") = " << q(i * 10 + 4) << ";" << endl;
        cout << "gamma_Old(" << i + 1 << ") = " << q(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "alpha_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 3) << ";" << endl;
        cout << "beta_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 4) << ";" << endl;
        cout << "gamma_New(" << i + 1 << ") = " << q0Dummy(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
        cout << "diff_alpha(" << i + 1 << ") = " << q(i * 10 + 3) - q0Dummy(i * 10 + 3) << ";" << endl;
        cout << "diff_beta(" << i + 1 << ") = " << q(i * 10 + 4) - q0Dummy(i * 10 + 4) << ";" << endl;
        cout << "diff_gamma(" << i + 1 << ") = " << q(i * 10 + 5) - q0Dummy(i * 10 + 5) << ";" << endl;
        cout << "%----------------------------------" << endl;
      }
    }

    setq0(q0Dummy);
    if (not filenameVel.empty())
      setu0(u0Dummy);

    if (DEBUGLEVEL > 0) {
      cout << "Positions = [ ";
      for (int ele = 0; ele < Elements; ele++) {
        MBSim::ContourPointData cp(ele);
        updateKinematicsForFrame(cp, MBSim::position_cosy);

        std::cout << cp.getFrameOfReference().getPosition()(0) << " " << cp.getFrameOfReference().getPosition()(1) << " " << cp.getFrameOfReference().getPosition()(2) << ";";
      }
      std::cout << "];" << std::endl;

      cout << "Normals = [ ";
      for (int ele = 0; ele < Elements; ele++) {
        MBSim::ContourPointData cp(ele);
        updateKinematicsForFrame(cp, MBSim::position_cosy);

        cout << cp.getFrameOfReference().getOrientation()(0, 0) << " " << cp.getFrameOfReference().getOrientation()(0, 1) << " " << cp.getFrameOfReference().getOrientation()(0, 2) << ";";
      }
      cout << "];" << endl;

      cout << "Tangents = [ ";
      for (int ele = 0; ele < Elements; ele++) {
        MBSim::ContourPointData cp(ele);
        updateKinematicsForFrame(cp, MBSim::position_cosy);

        cout << cp.getFrameOfReference().getOrientation()(1, 0) << " " << cp.getFrameOfReference().getOrientation()(1, 1) << " " << cp.getFrameOfReference().getOrientation()(1, 2) << ";";
      }
      cout << "];" << endl;

      cout << "Binormals = [ ";
      for (int ele = 0; ele < Elements; ele++) {
        MBSim::ContourPointData cp(ele);
        updateKinematicsForFrame(cp, MBSim::position_cosy);

        cout << cp.getFrameOfReference().getOrientation()(2, 0) << " " << cp.getFrameOfReference().getOrientation()(2, 1) << " " << cp.getFrameOfReference().getOrientation()(2, 2) << ";";
      }
      cout << "];" << endl;
    }

#else
    throw MBSimError("No Nurbs-Library installed ...");
#endif
  }

}

#endif /* _FLEXIBLE_BODY_1S_33_RCM_H_ */


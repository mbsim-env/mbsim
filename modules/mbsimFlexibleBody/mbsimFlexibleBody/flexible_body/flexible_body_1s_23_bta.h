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
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_1S_23_BTA_H_
#define _FLEXIBLE_BODY_1S_23_BTA_H_

#include "mbsimFlexibleBody/flexible_body.h"
#include "mbsimFlexibleBody/contours/cylinder_flexible.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#endif

namespace MBSimFlexibleBody {

  /*! 
   * \brief bending torsional axis
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-11-22 initial commit kernel_dev
   * \todo gravity, handling for contour-node information, tangents and AWK TODO
   */
  class FlexibleBody1s23BTA : public FlexibleBodyContinuum<double> {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody1s23BTA(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody1s23BTA() {}

      /* INHERITED INTERFACE OF FLEXIBLE BODY */
      virtual void BuildElements();
      virtual void GlobalVectorContribution(int n, const fmatvec::Vec& locVec, fmatvec::Vec& gloVec);
      virtual void GlobalMatrixContribution(int n, const fmatvec::Mat& locMat, fmatvec::Mat& gloMat);
      virtual void GlobalMatrixContribution(int n, const fmatvec::SymMat& locMat, fmatvec::SymMat& gloMat);
      virtual void updateKinematicsForFrame(MBSim::ContourPointData &cp, MBSim::FrameFeature ff, MBSim::Frame *frame=0);
      virtual void updateJacobiansForFrame(MBSim::ContourPointData &data, MBSim::Frame *frame=0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECT */
      virtual void init(MBSim::InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual void plot(double t, double dt=1);
      virtual std::string getType() const { return "FlexibleBody1s23BTA"; }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief sets size of positions and velocities
       */
      void setNumberElements(int n); 
      void setLength(double L_) { L = L_; }
      void setElastModuls(double E_, double G_) { E = E_;G = G_; }
      void setDensity(double rho_) { rho = rho_; }
      void setCrossSectionalArea(double A_) { A = A_; }
      void setMomentsInertia(double Iyy_,double Izz_,double It_) { Iyy = Iyy_; Izz = Izz_; It = It_; }
      void setContourRadius(double r) { cylinderFlexible->setRadius(r); }
      void setTorsionalDamping(double d) { dTorsional = d; }
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpineExtrusion(OpenMBV::SpineExtrusion* body) { openMBVBody=body; }
#endif
      /***************************************************/

    protected:
      /**
       * \brief detect current finite element
       * \param global parametrisation
       * \param local parametrisation
       * \param finite element number
       */
      void BuildElement(const double& sGlobal, double& sLocal, int& currentElement);

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
      double Iyy, Izz, It;

      /**
       * \brief density 
       */
      double rho;

      /**
       * \brief contour radius
       */
      double rc;

      /**
       * \brief damping 
       */
      double dTorsional;

      /** 
       * \brief contour of body
       */
      CylinderFlexible *cylinderFlexible;
  };

}

#endif /* _FLEXIBLE_BODY_1S_23_BTA_H_ */


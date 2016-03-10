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
 */

#ifndef NURBSDISK2S_H_
#define NURBSDISK2S_H_

#include "fmatvec/fmatvec.h"
#include "mbsim/mbsim_event.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/contours/contour2s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#endif

#ifdef HAVE_NURBS
#include "nurbs++/nurbs.h"
#include "nurbs++/nurbsS.h"
#include "nurbs++/vector.h"
#endif

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {

  /*!  
   * \brief 2s flexible
   * \author Kilian Grundl
   * \author Raphael Missel
   * \author Thorsten Schindler
   * \date 2009-05-22 initial commit (Grundl / Missel / Schindler)
   * \date 2009-06-04 separate contour files (Thorsten Schindler)
   * \date 2009-08-16 contour / visualisation (Grundl / Missel / Schindler)
   * \date 2010-04-21 flexible disks with parent (Grundl / Schindler)
   * \todo computeSurfaceJacobians / computeSurfaceVelocities only in contact case TODO
   * \todo angularVelocity TODO
   * \todo flexible body should only parametrise midplane -> other surfaces in contour TODO
   */
  class NurbsDisk2s : public MBSim::Contour2s {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      NurbsDisk2s(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~NurbsDisk2s();  

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "NurbsDisk2s"; }
      /***************************************************/

      void init(InitStage stage);

      virtual MBSim::ContourFrame* createContourFrame(const std::string &name="P");

      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta); }
      virtual fmatvec::Vec3 getWv(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta); }
      virtual fmatvec::Vec3 getWn(double t, const fmatvec::Vec2 &zeta);

      void updatePositions(double t, MBSim::ContourFrame *frame);
      void updateVelocities(double t, MBSim::ContourFrame *frame);
      void updateAccelerations(double t, MBSim::ContourFrame *frame);
      void updateJacobians(double t, MBSim::ContourFrame *frame, int j=0);
      void updateGyroscopicAccelerations(double t, MBSim::ContourFrame *frame);

      fmatvec::Vec3 getPosition(double t);
      fmatvec::SqrMat3 getOrientation(double t);

      virtual void plot(double t, double dt=1);

      virtual MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) { return findContactPairingFlexible(type0.c_str(), type1.c_str()); }

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        openMBVNurbsDisk = OpenMBV::ObjectFactory::create<OpenMBV::NurbsDisk>();
      }
      boost::shared_ptr<OpenMBV::Group> getOpenMBVGrp() { return openMBVGrp; }
#endif

//#ifdef HAVE_NURBS
//      /**
//       * \brief initialize NURBS disk
//       * \param stage of initialisation
//       */
//      void initContourFromBody(InitStage stage);
//#endif

      /*! 
       * \brief transformation cartesian to cylinder system
       * \param cartesian vector in world system
       * \return cylindrical coordinates
       */
      fmatvec::Vec transformCW(const fmatvec::Vec& WrPoint);

#ifdef HAVE_NURBS
      /*! 
       * \return derivates of the surface: (first column: deg-th derivative in radial direction / second column: deg-th derivative in azimuthal-direction)
       * \param radial location
       * \param azimuthal location
       * \param order of derivative
       */
      fmatvec::Mat computeDirectionalDerivatives(const double &radius, const double &phi, const int &deg);
#endif

#ifdef HAVE_NURBS
      /*!
       * \return curvature on the surface: (first column: radial direction / second column: azimuthal-direction)
       * \param radial location
       * \param azimuthal location
       */
      fmatvec::Mat computeCurvatures(const double &radius, const double &phi);
#endif

#ifdef HAVE_NURBS
      /*! 
       * \brief computes the U vector of the surface for a closed interpolation
       * \param
       */
      void computeUVector(const int NbPts); 
#endif

#ifdef HAVE_NURBS
      /*! 
       * \brief computes the V-vector of the surface for an open interpolation
       * \param
       */
      void computeVVector(const int NbPts);
#endif

#ifdef HAVE_NURBS
      /*! 
       * \brief interpolates the surface with node-data from body
       */
      void computeSurface(double t);
#endif

#ifdef HAVE_NURBS
      /*!
       * \brief interpolates the velocities of the surface with the node-data from the body
       */
      void computeSurfaceVelocities(double t);
#endif

#ifdef HAVE_NURBS
      /*! 
       * \brief interpolates the Jacobians of translation of the surface with the node-data from the body
       */
      void computeSurfaceJacobians(double t);
#endif

#ifdef HAVE_NURBS
      /*!
       * \return control point
       * \param u location
       * \param v location
       */
      fmatvec::Vec getControlPoints(const int u, const int v);
#endif

#ifdef HAVE_NURBS
      /*! 
       * return U-Vector of the surface (azimuthal direction)
       */
      fmatvec::Vec getUVector();
#endif

#ifdef HAVE_NURBS
      /*! 
       * return V-Vector of the surface (radial direction)
       */
      fmatvec::Vec getVVector();
#endif

      /*! 
       * \return flag, whether the input radius is inside the bounds or the input angle is between 0 and 2 PI
       * \param parametrisation vector
       */
      int testInsideBounds(const fmatvec::Vec &s);

      /*! 
       * \return norm of the difference between two vectors
       * \param first vector
       * \param second vector
       */
      double computeError(const fmatvec::Vec &Vec1, const fmatvec::Vec &Vec2);

      MBSim::Frame* getFrame(const std::string &name, bool check=true) const;

      void addFrame(MBSim::Frame *frame);

      void resetUpToDate();

    protected:
      /** 
       * \brief number of reference dofs of the flexible body
       */
      int RefDofs;

      /**
       * \brief number of elements in azimuthal and radial direction
       */
      int nj, nr;

      /**
       * \brief interpolation degree azimuthal and radial
       */
      int degU, degV;

      /**
       * \brief inner and outer radius
       */
      double Ri, Ra;

      std::vector<MBSim::Frame*> frame;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::NurbsDisk> openMBVNurbsDisk;
      boost::shared_ptr<OpenMBV::Group> openMBVGrp;
      double drawDegree;
#endif

#ifdef HAVE_NURBS
      /**
       * \brief Jacobians of finite element nodes
       */
      std::vector<NodeFrame> jacobians; // size = number of interpolation points

      /** 
       * \brief interpolated surface of the contour
       */
      PlNurbsSurfaced *Surface; // an homogenous surface with accuracy of calculation of double (HSurface<double,3>)

      /** 
       * \brief interpolated velocities of the surface-points
       */
      PlNurbsSurfaced *SurfaceVelocities;

      /**
       * \brief interpolated Jacobians of Translation on the surface 
       */
      std::vector<PlNurbsSurfaced> SurfaceJacobiansOfTranslation; // size = number of generalized coordinates

      /**
       * \brief interpolated Jacobians of Rotation on the surface
       */
      std::vector<PlNurbsSurfaced> SurfaceJacobiansOfRotation; // size = number of generalized coordinates

      /**
       * \brief knot vectors, used for the U und V coordinates of the surface
       */
      PLib::Vector<double> *uvec; // nurbs++ needs this vector 
      PLib::Vector<double> *uVec; // knot-vector for azimuthal direction 
      PLib::Vector<double> *vvec; // nurbs++ needs this vector
      PLib::Vector<double> *vVec; // knot-vector for radial direction
#endif
  };

}

#endif /* NURBSDISK2S_H_ */

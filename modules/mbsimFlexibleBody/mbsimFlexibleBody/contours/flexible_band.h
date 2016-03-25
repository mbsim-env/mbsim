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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _FLEXIBLE_BAND_H_
#define _FLEXIBLE_BAND_H_

#include "mbsimFlexibleBody/contours/contour1s.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#endif

#include <list>

namespace MBSim {

  BOOST_PARAMETER_NAME(numberOfSpinePoints)

}

namespace MBSimFlexibleBody {

  class FlexibleBand : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      FlexibleBand(const std::string& name) : Contour1s(name), width(0), ARK(fmatvec::EYE), sOld(-1e12) { }

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "FlexibleBand"; }
      void init(InitStage stage);
     /***************************************************/

      virtual MBSim::ContourFrame* createContourFrame(const std::string &name="P");

      /* GETTER / SETTER */
      void setWidth(double width_) { width = width_; }
      double getWidth() const { return width; }

      void setRelativePosition(const fmatvec::Vec2 &r);
      void setRelativeOrientation(double al);

      const fmatvec::Vec3& getRelativePosition() const { return RrRP; }
      const fmatvec::SqrMat3& getRelativeOrientation() const { return ARK; }

      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta) { return getPosition(t,zeta(0)); }
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta(0)); }
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta(0)); }
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta) { return getWs(t,zeta); }
      virtual fmatvec::Vec3 getWv(double t, const fmatvec::Vec2 &zeta) { return getWt(t,zeta); }

      virtual bool isZetaOutside(const fmatvec::Vec2 &zeta) { return zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1] or zeta(1) < -0.5*width or zeta(1) > 0.5*width; }

      void updatePositions(double t, double s);

      fmatvec::Vec3 getPosition(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return WrOP; }
      fmatvec::Vec3 getWs(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return Ws; }
      fmatvec::Vec3 getWt(double t, double s) { if(fabs(s-sOld)>MBSim::macheps()) updatePositions(t,s); return Wt; }

      virtual void plot(double t, double dt=1);

      void setContourOfReference(Contour1s *contour_) { contour = contour_; }

      MBSim::ContactKinematics * findContactPairingWith(std::string type0, std::string type1) { return findContactPairingFlexible(type0.c_str(), type1.c_str()); }

      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (numberOfSpinePoints,(int),10)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        openMBVSpineExtrusion = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
        openMBVSpineExtrusion->setNumberOfSpinePoints(numberOfSpinePoints);
      }
#endif

      void resetUpToDate();

    protected:
      /**
       * \brief width of flexible band
       */
      double width;

      fmatvec::Vec3 RrRP, WrOP, Ws, Wt;
      fmatvec::SqrMat3 ARK;

      Contour1s* contour;

      double sOld;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::SpineExtrusion> openMBVSpineExtrusion;
#endif
  };

}

#endif /* _FLEXIBLE_BAND_H_ */

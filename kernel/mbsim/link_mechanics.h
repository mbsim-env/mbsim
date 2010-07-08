/* Copyright (C) 2004-2009 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _LINK_MECHANICS_H_
#define _LINK_MECHANICS_H_

#include "link.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Group;
  class Arrow;
}
#endif

namespace H5 {
  class Group;
}

namespace MBSim {
  class Contour;

  /** 
   * \brief general link to one or more objects
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration improvement (Thorsten Schindler)
   * \date 2009-08-19 fix in dhdu referencing (Thorsten Schindler)
   */
  class LinkMechanics : public Link {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      LinkMechanics(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~LinkMechanics();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updatedhdz(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Link"; }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, const fmatvec::Vec &hLinkRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0);
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \param frame to add to link frame vector
       */
      virtual void connect(Frame *frame_);

      /**
       * \param contour to add to link contour vector
       */
      virtual void connect(Contour *contour_);

      /**
       * \brief TODO
       */
      virtual void resizeJacobians(int j) {}

    protected:
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVForceArrow(OpenMBV::Arrow *arrow, const std::vector<bool>& which);
      void setOpenMBVMomentArrow(OpenMBV::Arrow *arrow, const std::vector<bool>& which);
#endif

      /** 
       * \brief force and moment direction for smooth right hand side
       */
      std::vector<fmatvec::Vec> WF, WM;

      /**
       * \brief cartesian force and moment direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> fF, fM;

      /**
       * \brief array in which all frames are listed, connecting bodies via a link
       */
      std::vector<Frame*> frame;

      /** 
       * \brief array in which all contours are listed, connecting bodies via link
       */
      std::vector<Contour*> contour;

#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group *openMBVForceGrp;
      std::vector<OpenMBV::Arrow*> openMBVArrowF;
      std::vector<OpenMBV::Arrow*> openMBVArrowM;
#endif
  };
}

#endif /* _LINK_MECHANICS_H_ */


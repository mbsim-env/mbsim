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

namespace H5 {
  class Group;
}

#ifdef HAVE_AMVIS
namespace AMVis { class Arrow; }
#endif

namespace MBSim {
  class Frame;
  class Contour;
  class UserFunction;

  /** 
   * \brief general link to one or more objects
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included (Thorsten Schindler)
   * \todo delete load/save TODO
   */
  class LinkMechanics : public Link {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      LinkMechanics(const std::string &name) : Link(name) {}

      /**
       * \brief destructor
       */
      virtual ~LinkMechanics();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      //void load(const std::string& path, std::ifstream &inputfile);
      //void save(const std::string &path, std::ofstream &outputfile);
      std::string getType() const { return "Link"; }
      virtual void plot(double t, double dt = 1);
      /***************************************************/
      
      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatehRef(const fmatvec::Vec &ref, int i=0);
      virtual void updaterRef(const fmatvec::Vec &ref);
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \brief plots time series header
       */
      virtual void initPlot();

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

      /**
       * \param arrow do display the link load (force or moment)
       * \param scale scalefactor (default=1): scale=1 means 1KN or 1KNM is equivalent to arrowlength one
       * \param ID of load and corresponding frame/contour (ID=0 or 1)
       * \param userfunction to manipulate color of arrow at each timestep (default: red arrow for forces and green one for moments)
       */
#ifdef HAVE_AMVIS
      virtual void addAMVisForceArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
      virtual void addAMVisMomentArrow(AMVis::Arrow *arrow,double scale=1, int ID=0, UserFunction *funcColor=0);
#endif
      /***************************************************/

    protected:
      /** 
       * \brief force and moment direction for smooth right hand side
       */
      std::vector<fmatvec::Vec> WF, WM;
      
      /**
       * \brief force and moment direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> fF, fM;

      /**
       * \brief array in which all frames are listed, connecting bodies via a link
       */
      std::vector<Frame*> port;

      /** 
       * \brief array in which all contours are listed, connecting bodies via link
       */
      std::vector<Contour*> contour;

#ifdef HAVE_AMVIS
      std::vector<AMVis::Arrow*> arrowAMVis;
      std::vector<double> arrowAMVisScale;
      std::vector<int> arrowAMVisID;
      std::vector<bool> arrowAMVisMoment;
      std::vector<UserFunction*> arrowAMVisUserFunctionColor;
#endif
  };
}

#endif /* _LINK_MECHANICS_H_ */


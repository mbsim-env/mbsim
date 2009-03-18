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
 *          rzander@users.berlios.de
 */

#ifndef _FLEXIBLE_BODY_H_
#define _FLEXIBLE_BODY_H_

#include <vector>
#include <mbsim/body.h>

#ifdef HAVE_AMVIS
namespace AMVis {class ElasticBody;}
#endif

namespace MBSim {

  class ContourPointData;

  /**
   * \brief upmost class for flexible body implementation
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 17.03.09
   */
  class FlexibleBody : public Body {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody(const string &name); 

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody();

      /* INHERITED INTERFACE */
      /* OBJECTINTERFACE */
      /** 
       * \brief compute mass matrix and smooth right hand side
       * \param time
       */
      virtual void updatedq(double t, double dt) { qd = u*dt; }
      virtual void updateqd(double t) { qd = u; }
      virtual void updateh(double t);
      virtual void updateKinematics(double t);
      virtual void updateJacobians(double t);
      virtual void updateSecondJacobians(double t); 

      /* ELEMENT */
      virtual void plot(double t, double dt=1, bool top=true);
      virtual void initPlot(bool top=true);
      virtual string getType() const { return "FlexibleBody"; }
      virtual void load(const string &path, ifstream& inputfile) { Body::load(path, inputfile); }
      virtual void save(const string &path, ofstream &outputfile) { Body::save(path, outputfile); }

      /* OBJECT */
      virtual void updateqRef(Vec q_) { q  >> q_ ; }
      virtual void updateqdRef(Vec qd_) { qd >> qd_; }
      virtual void updateuRef(Vec u_) { u  >> u_ ; }
      virtual void updateudRef(Vec ud_) { ud >> ud_; }
      virtual void updatehRef(Vec h_) { h  >> h_ ; }
      virtual void updaterRef(Vec r_) { r  >> r_ ; }
      virtual void updateMRef(SymMat M_) { M  >> M_ ; }
      // TODO TRef, LLMRef?

      virtual void init();
      virtual double computePotentialEnergy();
      void setq0(Vec q0_) { Body::setq0(q0_); q>>q0; }
      void setu0(Vec u0_) { Body::setu0(u0_); u>>u0; }
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASSES */
      /** 
       * \brief insert 'local' information in global matrices
       * \param number of finite element
       */
      virtual void GlobalMatrixContribution(int CurrentElement) = 0;

      /**
       * \param contour parameters specifing location
       * \return JACOBIAN-matrix of system dimensions (size of positions x cartesian degree of freedom) 
       */
      virtual Mat computeJacobianMatrix(const ContourPointData &data) = 0; 

      /**
       * \param contour parameter
       * \return absolute position in world system to body contour
       */
      virtual Frame* computeKinematicsForFrame(const ContourPointData &data) = 0;
      /***************************************************/

      /**
       * \param stationary frame of reference
       */
      void setStationaryFrameOfReference(Frame *frame) { frameParent = frame; }

      /** 
       * \param mass proportional damping factor 
       */
      void setMassProportionalDamping(const double d_) { d_massproportional = d_; }

      /** 
       *  \param name of frame
       *  \param frame location
       */
      void addFrame(const string &name, const ContourPointData &S_);

      /**
       * \param name of frame
       * \param frame location
       */
      void addFrame(Frame *frame, const ContourPointData &S_);

#ifdef HAVE_AMVIS
      /** 
       * \brief activate output for AMVis
       * \param binary or ASCII data format in pos-file
       */
      void createAMVisBody(bool binary_) { boolAMVis = true; boolAMVisBinary = binary_; }

      /**
       * \param color float in [0;1] (blue - green - red)
       */
      void setAMVisColor(float color) { AMVisColor = color; }
#endif

    protected:
      /**
       * \brief inertial frame of reference of the flexible body
       */
      Frame *frameParent;

      /** 
       * \brief discretization
       */
      vector<DiscretizationInterface*> discretization;

      /** 
       * \brief finite element wise positions
       */
      vector<Vec> qElement;

      /** 
       * \brief finite element wise velocities
       */
      vector<Vec> uElement;

      /** 
       * \brief JACOBIAN-matrix of translations and rotations
       */
      Mat JT, JR;

      /**
       * \brief mass proportion damping factor
       */
      double d_massproportional;

      /** 
       * \brief vector of contour parameters each describing a frame
       */
      vector<ContourPointData> S_Frame;

      /**
       * \brief last computed frame for contour parameter
       */
      Frame *lastContourFrame;

#ifdef HAVE_AMVIS
      /** 
       * \brief body for AMVis
       */
      AMVis::ElasticBody *bodyAMVis;

      /** 
       * \brief flag to allow for activation of AMVis output during init-routines
       */
      bool boolAMVis, boolAMVisBinary;

      /**
       * \brief float to pass to AMVis-object
       */
      float AMVisColor;
#endif

  };

  /**
   * \brief flexible bodies defined by one material (or LAGRANGE) coordinate
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 17.03.09
   */
  class FlexibleBody1s : public FlexibleBody {
    public:
      /** 
       * \brief constructor
       * \param name of body
       */
      FlexibleBody1s(const string &name);

      virtual ~FlexibleBody1s() {}

      /**
       * \param nodes for search-fields of contact-search
       */
      void setContourNodes(const Vec& nodes) { userContourNodes = nodes; }

    protected:
      /**
       * \brief grid for contact point detection
       */
      Vec userContourNodes;

  };

  /**
   * \brief flexible bodies defined by two material (or LAGRANGE) coordinates
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 17.03.09
   */
  class FlexibleBody2s : public FlexibleBody {
    public:
      /**
       * \brief constructor
       * \param name of body
       */
      FlexibleBody2s(const string &name);

      /**
       * \brief destructor
       */
      virtual ~FlexibleBody2s() {}

      /**
       * \param nodes for search-fields of contact-search
       */
      void setContourNodes(const Mat& nodes) { userContourNodes = nodes; }

    protected:
      /**
       * \brief grid for contact point detection
       */
      Mat userContourNodes;
  };

}

#endif


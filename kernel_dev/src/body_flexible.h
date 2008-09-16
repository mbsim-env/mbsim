/* Copyright (C) 2005-2006  Roland Zander
 
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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#ifndef _BODY_FLEXIBLE_H_
#define _BODY_FLEXIBLE_H_

#include "body.h"
#include "contour_pdata.h"

#ifdef HAVE_AMVIS
namespace AMVis {class ElasticBody;}
#endif

namespace MBSim {

//  class Contour1sFlexible;
//  class Contour2sFlexible;

  /*! \brief Upmost class for flexible body implementation
   *
   * general definitions, common implementation e.g. for updateW(double t) and sumUpForceElements(double t)
   */
  class BodyFlexible : public Body {
    friend class TreeFlexRoot;
    friend class BodyRigidRelOnFlex;

    protected:
      /** JACOBIAN-matrizes: \f$\vJ_{ges}=(\vJ_T,\vJ_R)\f$ */
      Mat Jges;
      /** JACOBIAN-matrix of translations */
      Mat JT;
      /** JACOBIAN-matrix of translations */
      Mat JR;
      /** temporary martix of generalised load directions */
      Vec WLtmp;
      /** temporary martix of generalised force directions */
      Vec WFtmp;
      /** temporary martix of generalised moment directions */
      Vec WMtmp;

      /** vector of ContourPointData, e.g. contourparameters, each describing a CoordinateSystem */
      vector<ContourPointData> S_CoordinateSystem;
      /** specify wether Contour is native or added with specific S_Contour */
      vector<bool> constContourPosition;
      /** vector of ContourPointData, e.g. contourparameters, each describing a Contour */
      vector<ContourPointData> S_Contour;

      /* geerbt */
      void init();

      /*! compute \f$\dot{\vz}\f$
       *  \param t time of evaluation
       */
      void updatezd(double t);
      /*! compute \f$\Delta{\vu}\f$
       *  \param t  time of evaluation
       *  \param dt time step size
       */
      void updatedu(double t, double dt);
      /*! 
       * Compute \f$\Delta{\vq}\f$
       * \param t  time of evaluation
       * \param dt time step size
       */
      void updatedq(double t, double dt);
      /*! 
       * Compute \f$\dot{\vq}\f$
       * \param t  time of evaluation
       */
      void updateqd(double t) {qd << u;}

      /* geerbt */
      void updateT(double t) {}

      /** indices of forces in load vectors, finally defined classes in derived and specified classes/bodies */
      Index IndexForce;
      /** indices of moments in load vectors, finally defined classes in derived and specified classes/bodies */
      Index IndexMoment;

      /*! sum up forces acting on CoordinateSystem and Contour interfaces, collects JACOBIAN-matrizes of implementations
       * of computeJacobianMatrix() to distrubute loads
       *  \param t time of evaluation
       */
      virtual void sumUpForceElements(double t);

      /*! update generalised force directions \f$\vW\f$ for set-valued interactions on CoordinateSystem and Contour interfaces,
       *  therefor collects JACOBIAN-matrizes of implementations of computeJacobianMatrix()
       *  \param t  time of evaluation
       */
      void updateWj(double t);

#ifdef HAVE_AMVIS
      /** body for AMVis */
      AMVis::ElasticBody *bodyAMVis;
      /** flag to allow for activation of AMVis output during Init-Routines */
      bool boolAMVis, boolAMVisBinary;
      /** float to pass to AMVis-object */
      float AMVisColor;
#endif

    public:
      /*!  \param name  name of body */
      BodyFlexible(const string &name); 

      /*!  destructor mainly handles memory-cleanup */
      ~BodyFlexible(); 

      /*! set JACOBIAN-matrix BodyFlexible::JT, \f$\vJ_R\f$ of translations
       *  \param JT_ JACOBIAN-matrix of translations
       */
      void setJT(const Mat &JT_) {JT = JT_;}
      /*! set JACOBIAN-matrix BodyFlexible::JR, \f$\vJ_R\f$ of rotations
       *  \param JR_ JACOBIAN-matrix of rotations
       */
      void setJR(const Mat &JR_) {JR = JR_;}

#ifdef HAVE_AMVIS
      /*! activate output for AMVis
      */
      void createAMVisBody() {boolAMVis = true;  boolAMVisBinary = true;}
      /*! activate output for AMVis
	\param binary_ for binary or ASCII data format in pos-file
	*/
      void createAMVisBody(bool binary_) {boolAMVis = true; boolAMVisBinary = binary_;}
      /*! set color for AMVis-object
	\param color float; [0;1] blue - green - red; out of these bounds default color is used
	*/
      void setAMVisColor(float color) {AMVisColor = color;}
#endif

       /*! pre-definition for method giving JACOBIAN-matrix at contour-point S,
       *  needs to be defined in implementation for deduced classes
       *  @param S contour parameters specifing location
       *  \return JACOBIAN-matrix of system dimensions Object::qSize x BodyFlexible::Jges ->cols()
       */
      virtual Mat computeJacobianMatrix(const ContourPointData &data) = 0; // alle muessen!!! diese Methode zur Verfuegung stellen

      /*! definition for method giving numerical time-derivative of JACOBIAN-matrix at contour-point S,
       * should be re-defined in deduced classes providing \f$\boldsymbol{J}\f$ analytical
       *  @param S contour parameters specifing location
       *  \return \f$\dot{\boldsymbol{J}}\f$ of system dimensions Object::qSize x BodyFlexible::Jges ->cols()
       */
      virtual Mat computeJp(const ContourPointData &data);

      virtual Mat computeK    (const ContourPointData &cp);
      virtual Mat computeKp   (const ContourPointData &cp);
      virtual Mat computeDrDs (const ContourPointData &data);
      virtual Mat computeDrDsp(const ContourPointData &data);

      /*! compute matrix describing tangential directions to body contour
       * \param data contour parameter set
       */
      virtual Mat computeWt  (const ContourPointData &data) = 0;
      /*! compute vector describing normal direction to body contour at s
       * \param data contour parameter set
       */
      virtual Vec computeWn  (const ContourPointData &data) = 0;
      /*! compute trafo-matrix \f$\boldsymbol{A}_{WK}\f$ from contour to world system at s
       * \param data contour parameter
       */
      virtual SqrMat computeAWK (const ContourPointData &data) = 0;
      /*! compute time derivative \f$\dot{\boldsymbol{A}}_{WK} = \tilde{\boldsymbol{\omega}}_K\boldsymbol{A}_{WK}\f$ of \f$\boldsymbol{A}_{WK}\f$
       * \param data contour parameter
       */
      virtual SqrMat computeAWKp(const ContourPointData &data);

      /*! compute absolute position in world system to body contour at s
       * \param data contour parameter set
       */
       virtual Vec computeWrOC(const ContourPointData &data) = 0;
      /*! compute absolute velocity in world system to body contour at s
       * \param data contour parameter set
       */
      virtual Vec computeWvC (const ContourPointData &data) = 0;
      /*! compute absolute angular velocity in world of material point at alpha
       * \param s contour parameter
       */
      virtual Vec computeWomega(const ContourPointData& s) = 0;

      /*! return a CoordinateSystem, usable for Connection s, etc.
       * \param name used for access to CoordinateSystem
       * \return CoordinateSystem
       */
      virtual CoordinateSystem* getCoordinateSystem(const string &name);
      /*! define a CoordinateSystem
       * \param name used for access to CoordinateSystem
       * \param S_   contour parameter of port location
       */
      void addCoordinateSystem(CoordinateSystem *port, const ContourPointData &S_);
      /*! 
       *  \param name name of CoordinateSystem to access
       *  \return pointer to accessed CoordinateSystem 
       */
      void addCoordinateSystem(const string &name, const ContourPointData &S_);
      /*! add additional Contour
       * \param contour to add
       * \param S_ ContourPointData, used only in default case of constPosition=true
       * \param constPosition specify wether Contour uses constant ContourPointData on BodyFlexible
       */
      void addContour(Contour *contour, const ContourPointData &S_, bool constPosition=true);

      /*! write header for Element::plotfile */
      void initPlotFiles();
      /*! write generalised state of BodyFlexible, initially calls Element::plot(double t, double dt) to flush ostream and write time
       *  \param t  time of evaluation
       *  \param dt time step size
       */
      void plot(double t, double dt);
      /*! basic prototype plotting basic class name "BodyFlexible" */

      /* alle geerbt */
      void updateMRef(SymMat M_) { M  >> M_ ;}
      void updateqRef(Vec q_)    { q  >> q_ ;}
      void updateuRef(Vec u_)    { u  >> u_ ;}
      void updateqdRef(Vec qd_)  { qd >> qd_;}
      void updateudRef(Vec ud_)  { ud >> ud_;}
      void updatehRef(Vec h_)    { h  >> h_ ;}
      void updaterRef(Vec r_)    { r  >> r_ ;}
      void setq0(Vec q0_)        { Body::setq0(q0_); q>>q0; }
      void setu0(Vec u0_)        { Body::setu0(u0_); u>>u0; }
  };

  //####################################################
  /*! \brief Flexible bodies defined by one material (or LAGRANGE) coordinate */
  class BodyFlexible1s : public BodyFlexible {

    protected:
      /** grid for contact point detection given to Contour1sFlexible */
      Vec userContourNodes;

    public:
      /*! \param name  name of body */
      BodyFlexible1s(const string &name); 

      /*! add CoordinateSystem at Contour parameter s
       *  \param name of CoordinateSystem
       *  \param s    position of CoordinateSystem
       */
      void addCoordinateSystem(const string &name, const double &s);// { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,s); addCoordinateSystem(name,temp); }
      using BodyFlexible::addCoordinateSystem;
      /*! add additional Contour at specified position
       * \param contour Contour to add
       * \param s       position of Contour reference point
       */
      void addContour(Contour *contour, const double &s);// { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,s); addContour(contour,temp); }
      using BodyFlexible::addContour;

      //---------------------------------------------------------------------------
      using BodyFlexible::computeWt;
      using BodyFlexible::computeWn;
      using BodyFlexible::computeWrOC;
      using BodyFlexible::computeWvC;
      using BodyFlexible::computeWomega;

      /*! compute vector describing tangential direction to body contour at alpha */
      Mat computeWt  (const double &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,alpha); return computeWt(temp); }

      /*! compute vector describing normal direction to bodies contour at alpha */
      Vec computeWn  (const double &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,alpha); return computeWn(temp); }

      /*! compute position in world system of material point at alpha */
      Vec computeWrOC(const double &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,alpha); return computeWrOC(temp); }

      /*! compute absolute velocity in world of material point at alpha */
      Vec computeWvC (const double &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,alpha); return computeWvC(temp); }

      /*! compute absolute angular velocity in world of material point at alpha */
      Vec computeWomega(const double &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,alpha); return computeWomega(temp); }

      SqrMat computeAWK (const ContourPointData &data); 
//      SqrMat computeAWKp(const ContourPointData &data); 

       //---------------------------------------------------------------------------
      /*! definition of BodyFlexible1s::userContourNodes for search-fields of contact-search */
      void setContourNodes(const Vec& nodes) {userContourNodes = nodes;}
  };

  //####################################################
  /*! \brief Flexible bodies defined by two material (or LAGRANGE) coordinate */
  class BodyFlexible2s : public BodyFlexible {

	protected:
	  /** grid for contact point detection given to Contour1sFlexible */
	  Mat userContourNodes;

	public:
	  /*! \param name  name of body */
	  BodyFlexible2s(const string &name); 

      /*! add CoordinateSystem at Contour parameter s
       *  \param name of CoordinateSystem
       *  \param s    position of CoordinateSystem
       */
      void addCoordinateSystem(const string &name, const Vec &s);// { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,s); addCoordinateSystem(name,temp); }
      using BodyFlexible::addCoordinateSystem;
      /*! add additional Contour at specified position
       * \param contour Contour to add
       * \param s       position of Contour reference point
       */
      void addContour(Contour *contour, const Vec &s);// { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = Vec(1,INIT,s); addContour(contour,temp); }
      using BodyFlexible::addContour;

      //---------------------------------------------------------------------------
      using BodyFlexible::computeWt;
      using BodyFlexible::computeWn;
      using BodyFlexible::computeWrOC;
      using BodyFlexible::computeWvC;
      using BodyFlexible::computeWomega;

      /*! compute vector describing tangential direction to body contour at alpha */
      Mat computeWt  (const Vec &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = alpha; return computeWt(temp); }

      /*! compute vector describing normal direction to bodies contour at alpha */
      Vec computeWn  (const Vec &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = alpha; return computeWn(temp); }

      /*! compute position in world system of material point at alpha */
      Vec computeWrOC(const Vec &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = alpha; return computeWrOC(temp); }

      /*! compute absolute velocity in world of material point at alpha */
      Vec computeWvC (const Vec &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = alpha; return computeWvC(temp); }

      /*! compute absolute angular velocity in world of material point at alpha */
      Vec computeWomega(const Vec &alpha) { ContourPointData temp; temp.type = CONTINUUM; temp.alpha = alpha; return computeWomega(temp); }

      SqrMat computeAWK (const ContourPointData &data); 

///	  Vec computeWrOC(const Vec& s) {return computeWrOC(s,Vec("[1.0;0.0;0.0]"));}
///	  virtual Vec computeWrOC(const Vec& s, const Vec& Cb) = 0;
///	  virtual Vec transformCW(const Vec& WrPoint) = 0;

	  //---------------------------------------------------------------------------
	  /*! definition of BodyFlexible1s::userContourNodes for search-fields of contact-search */
	  void setContourNodes(const Mat& nodes) {userContourNodes = nodes;}
  };

}
#endif

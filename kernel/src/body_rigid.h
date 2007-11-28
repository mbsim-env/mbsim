/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _BODY_RIGID_H_
#define _BODY_RIGID_H_

#include "body.h"
#include "fmatvec.h"
#include <vector>

#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif

namespace MBSim {
  class DataInterfaceBase;

  enum Rot {cardanAngles, eulerParameters};

  /*! \brief Class for rigid bodies 
   *
   * */
  class BodyRigid : public Body {

    protected:
      /** mass */
      double m;
      SymMat I, Mh;
      Vec WrOK, WvK, WomegaK, KomegaK, KrKS;
      SqrMat AWK, AK0K;
      Mat H, TH;

      Vec l, WF, WM, WLtmp, WFtmp, WMtmp;
      Mat J, JT, JR;
      Index iT, iR;

      Rot rot;

      vector<SqrMat> AKC;
      vector<Vec> KrKP, WrKP, KrKC, WrKC;

      virtual void updateKinematics(double t);
      virtual void updatePorts(double t);
      virtual void updateContours(double t);
      virtual void updateCenterOfGravity(double t) = 0;
      virtual void sumUpForceElements(double t);
      virtual void updatezd(double t);
      virtual void updatedu(double t, double dt);
      virtual void updatedq(double t, double dt);
      void updateT(double t) {(this->*updateT_)();}

      void (BodyRigid::*updateAK0K)();
      void (BodyRigid::*updateT_)();
      void noUpdateAK0K() {}
      void noUpdateT() {}
      void updateAK0KAxis();
      void updateAK0KCardanAngles();
      void updateTCardanAngles();
      void updateAK0KEulerParameters();
      void updateTEulerParameters();

      bool inertiaWithRespectToCOG;

#ifdef HAVE_AMVIS
      AMVis::CRigidBody *bodyAMVis;
      DataInterfaceBase* bodyAMVisUserFunctionColor;
      bool AMVisDataRel;
#endif

    public:

      BodyRigid(const string &name);
      virtual ~BodyRigid();

      /*! define Jacobian of translations in world system
	\param JT Jacobian-matrix
	*/
      void setJT(const Mat &JT);
      /*! define Jacobian of rotations in world system
	\param JR Jacobian-matrix
	*/
      void setJR(const Mat &JR);

#ifdef HAVE_AMVIS
      void setAMVisBody(AMVis::CRigidBody *body, DataInterfaceBase* funcColor=0) {bodyAMVis= body; bodyAMVisUserFunctionColor= funcColor;}
      /*! set output to center of reference (true) or center of gravity (false, default)
	\param rel_
       */
      void setAMVisOutputRel(bool rel_) {AMVisDataRel = rel_;}
#endif

      Vec computeJTqT() const {return JT*q(iT);}
      Vec computeJRqR() const {return JR*q(iR);}

      const Vec computeWrOS() const {return WrOK + AWK*KrKS;}

      const Mat& getJT() const {return JT;}
      const Mat& getJR() const {return JR;}

      void plot(double t, double dt = 1);
      void plotParameters();
      void initPlotFiles();
      void init();
      void calcSize();

      const SqrMat& getAWK() const {return AWK;}
      const Vec& getWvK() const {return WvK;}
      const Vec& getKomegaK() const {return KomegaK;}

      double computePotentialEnergy();//{return - m * trans(mbs->getGrav()) * WrOS;}
      //double computeKineticEnergy();

      /*! define the mass of the body
	\param m mass
	*/
      void setMass(double m_) {m = m_;}

      /*! \brief matrix of inertia
       * define the matrix of inertia with respect to the point of reference if
       * cog = false. If cog = true the inertia has to be defined with respect to the center of gravity
	\param I martix of inertia
	*/
      void setInertia(const SymMat& I_, bool cog = false) {I = I_; inertiaWithRespectToCOG = cog;}

      void setKrKS(const Vec& KrKS_) {KrKS = KrKS_;}
      const Vec& getKrKS() const {return KrKS;}

      void setRotationalParameters(Rot rot_) {rot = rot_;}

      void addPort(Port * port, const Vec &KrKP);
      /*! add a Port with constant relativ position in body-fixed system K
      */
      void addPort(const string &name, const Vec &KrKP);

      void addContour(Contour* contour, const Vec &KrKC, const SqrMat &AKC);
      void addContour(Contour *contour, const Vec &KrKC);
      void addContour(Contour *contour);

  };

}

#endif


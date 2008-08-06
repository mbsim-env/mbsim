/* Copyright (C) 2004-2008  Martin Förg
 
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

#ifndef _RIGID_BODY_H_
#define _RIGID_BODY_H_

#include "body.h"
#include "fmatvec.h"
#include <vector>
#include "tree.h"
#include "coordinate_system.h"
#include "userfunction.h"
#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif

// Projektive Newton-Euler-Gleichungen bzgl. COG


namespace MBSim {

  class Translation {
    public:
      virtual ~Translation() {}
      virtual int getqSize() const = 0;
      virtual Vec operator()(const Vec &q, double t) = 0; 
  };

  class LinearTranslation : public Translation {
    private:
      Mat PJT;
    public:
      LinearTranslation(const Mat &PJT_) { PJT = PJT_; } 

      const Mat& getPJT() const {return PJT;}

      virtual int getqSize() const {return PJT.cols();}

      virtual Vec operator()(const Vec &q, double t) {
	return PJT*q(0,PJT.cols()-1);
      }; 
  };

  class Rotation {
    public:
      virtual ~Rotation() {}
      virtual int getqSize() const = 0;
      virtual SqrMat operator()(const Vec &q, double t) = 0; 
  };

  class Jacobian {
    public:
      virtual ~Jacobian() {}
      virtual int getuSize() const = 0;
      virtual Mat operator()(const Vec &q, double t) = 0; 
  };

  class ConstJacobian : public Jacobian {
    private:
      Mat J;
    public:
      virtual int getuSize() const {return J.cols();}
      ConstJacobian(const Mat &J_) {J = J_;}

    public:
       virtual Mat operator()(const Vec &q, double t) {return J;} 
  };

  class DerJac {
    public:
      virtual ~DerJac() {}
      virtual Mat operator()(const Vec &qd, const Vec &q, double t) = 0; 
  };

  class TimeDep {
    public:
      virtual ~TimeDep() {}
      virtual Vec operator()(double t) = 0; 
  };


  class RotationAxis: public Rotation {
    private:
      SqrMat APK;
      Vec a;
    public:
      RotationAxis() : APK(3), a(3) {}
      RotationAxis(const Vec &a_) : APK(3) { a = a_; } 

      const Vec& getAxis() const {return a;}
      virtual int getqSize() const {return 1;}

      void setAxis(const Vec& a_) {a = a_;}

      SqrMat operator()(const Vec &q, double t) {
	int i = q.size()-1;
	double cosq=cos(q(i));
	double sinq=sin(q(i));
	double onemcosq=1-cosq;
	double a0a1=a(0)*a(1);
	double a0a2=a(0)*a(2);
	double a1a2=a(1)*a(2);
	APK(0,0) = cosq+onemcosq*a(0)*a(0);
	APK(1,0) = onemcosq*a0a1+a(2)*sinq;
	APK(2,0) = onemcosq*a0a2-a(1)*sinq;
	APK(0,1) = onemcosq*a0a1-a(2)*sinq;
	APK(1,1) = cosq+onemcosq*a(1)*a(1);
	APK(2,1) = onemcosq*a1a2+a(0)*sinq;
	APK(0,2) = onemcosq*a0a2+a(1)*sinq;
	APK(1,2) = onemcosq*a1a2-a(0)*sinq;
	APK(2,2) = cosq+onemcosq*a(2)*a(2);
	return APK;
      }
  };

  class CardanAngles: public Rotation {
    private:
      SqrMat APK;
      Vec a;
    public:
      CardanAngles() : APK(3), a(3) {}
      CardanAngles(const Vec &a_) : APK(3) { a = a_; } 

      virtual int getqSize() const {return 3;}

      SqrMat operator()(const Vec &q, double t) {
	int i = q.size()-1;
	double a=q(i-2);
	double b=q(i-1);
	double g=q(i);

	APK(0,0) = cos(b)*cos(g);
	APK(1,0) = sin(a)*sin(b)*cos(g)+cos(a)*sin(g);
	APK(2,0) = -cos(a)*sin(b)*cos(g)+sin(a)*sin(g);
	APK(0,1) = -cos(b)*sin(g);
	APK(1,1) = -sin(g)*sin(b)*sin(a)+cos(a)*cos(g);
	APK(2,1) = cos(a)*sin(b)*sin(g)+sin(a)*cos(g);
	APK(0,2) = sin(b);
	APK(1,2) = -sin(a)*cos(b);
	APK(2,2) = cos(a)*cos(b);
	return APK;
      }
  };

  class TCardanAngles : public Jacobian {
    private:
      int qSize, uSize;
      Mat T;

    public:
      TCardanAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize) {
	for(int i=0; i<uSize; i++)
	  T(i,i) = 1;
      }
      int getuSize() const {return uSize;}
       virtual Mat operator()(const Vec &q, double t) {
	  int i = q.size()-1;
	  double alpha = q(i-2);
	  double beta = q(i-1);
	  double cos_beta = cos(beta);
	  double sin_beta = sin(beta);
	  double cos_alpha = cos(alpha);
	  double sin_alpha = sin(alpha);
	  double tan_beta = sin_beta/cos_beta;

	  T(i-2,i-1) =    tan_beta*sin_alpha;
	  T(i-2,i) =   -tan_beta*cos_alpha;
	  T(i-1,i-1) =  cos_alpha;
	  T(i-1,i) =  sin_alpha;
	  T(i,i-1) = -sin_alpha/cos_beta;           
	  T(i,i) =  cos_alpha/cos_beta;
	  return T;
       }
  };

  class TCardanAngles2 : public Jacobian {
    private:
      int qSize, uSize;
      Mat T;

    public:
      TCardanAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize) {
	for(int i=0; i<uSize; i++)
	  T(i,i) = 1;
      }
      int getuSize() const {return uSize;}
      virtual Mat operator()(const Vec &q, double t) {
	int i = q.size()-1;
	double beta = q(i-1);
	double gamma = q(i);
	double cos_beta = cos(beta);
	double sin_beta = sin(beta);
	double cos_gamma = cos(gamma);
	double sin_gamma = sin(gamma);
	double tan_beta = sin_beta/cos_beta;

	T(i-2,i-2) = cos_gamma/cos_beta;
	T(i-2,i-1) = -sin_gamma/cos_beta;
	T(i-1,i-2) = sin_gamma;
	T(i-1,i-1) = cos_gamma;
	T(i,i-2) = -cos_gamma*tan_beta;
	T(i,i-1) = sin_gamma*tan_beta;           
	return T;
       }
  };

//  class PJRTest : public Jacobian {
//    private:
//      Mat KJR;
//      CoordinateSystem *portRef, portParent;
//
//    public:
//      PJRTest(CoordinateSystem* portRef_,CoordinateSystem* portParent_, const Mat &KJR_) : portRef(portRef_), portParent(portParent_) {
//	KJR = KJR_;
//      }
//      int getuSize() const {return KJR.cols();}
//
//      virtual Mat operator()(const Vec &q, double t) {
//	return trans(portParent->getAWP()*portRef->getAWP()*KJR;
//	return port->getAWP()*KJR;
//       }
//  };

  /*! \brief Class for rigid bodies with relative coordinates 
   *
   * */
  class RigidBody : public Body {

    friend class TreeTest;

    protected:
    bool cb;
    double m;
    SymMat SThetaS, WThetaS;
    int iRef, i4I;
    Mat H, TH;
    SymMat Mbuf;

    Mat PJT, PJR, PdJT, PdJR;
    Vec PjT, PjR, PdjT, PdjR;

    Mat PJR0;

    SqrMat APK;
    Vec PrPK, WrPK, WvPKrel, WomPK;
    CoordinateSystem *portParent;
    vector<SqrMat> ASK;
    vector<Vec> SrSK, WrSK;

    vector<SqrMat> ASC;
    vector<Vec> SrSC, WrSC;

    Jacobian *fT;

    Translation *fPrPK;
    Rotation *fAPK;
    Jacobian *fPJT;
    Jacobian *fPJR;
    DerJac *fPdJT;
    DerJac *fPdJR;
    TimeDep *fPjT;
    TimeDep *fPjR;
    TimeDep *fPdjT;
    TimeDep *fPdjR;

    virtual void updateh(double t);
    virtual void updateJacobians(double t);
    virtual void updateKinematics(double t);

    void updateM(double t) {(this->*updateM_)(t);}
    void (RigidBody::*updateM_)(double t);
    void updateMConst(double t);
    void updateMNotConst(double t); 

    void facLLM() {(this->*facLLM_)();}
    void (RigidBody::*facLLM_)();
    void facLLMConst() {};
    void facLLMNotConst() {Object::facLLM();}

    void updateT(double t) {if(fT) T = (*fT)(q,t);}

#ifdef HAVE_AMVIS
      AMVis::CRigidBody *bodyAMVis;
      DataInterfaceBase* bodyAMVisUserFunctionColor;
      CoordinateSystem* cosyAMVis;
#endif


    public:
    RigidBody(const string &name);

    void useCoordinateSystemOfBodyForRotation(bool cb_) {cb = cb_;}
    void setTranslation(Translation* fPrPK_) { fPrPK = fPrPK_;}
    void setRotation(Rotation* fAPK_) { fAPK = fAPK_;}
    void setJacobianOfTranslation(Jacobian* fPJT_) { fPJT = fPJT_;}
    void setJacobianOfRotation(Jacobian* fPJR_) { fPJR = fPJR_;}
    void setDerivativeOfJacobianOfTranslation(DerJac* fPdJT_) { fPdJT = fPdJT_;}
    void setDerivativeOfJacobianOfRotation(DerJac* fPdJR_) { fPdJR = fPdJR_;}
    void setGuidingVelocityOfTranslation(TimeDep* fPjT_) { fPjT = fPjT_;}
    void setGuidingVelocityOfRotation(TimeDep* fPjR_) { fPjR = fPjR_;}
    void setDerivativeOfGuidingVelocityOfTranslation(TimeDep* fPdjT_) { fPdjT = fPdjT_;}
    void setDerivativeOfGuidingVelocityOfRotation(TimeDep* fPdjR_) { fPdjR = fPdjR_;}

    /*! define the mass of the body
      \param m mass
      */
    void setMass(double m_) {m = m_;}

    /*! \brief matrix of inertia
     * define the matrix of inertia with respect to the point of reference if
     * cog = false. If cog = true the inertia has to be defined with respect to the center of gravity
     \param I martix of inertia
     */
    void setMomentOfInertia(const SymMat& RThetaR, const CoordinateSystem* refCoordinateSystem=0) {
      if(refCoordinateSystem)
	i4I = portIndex(refCoordinateSystem);
      else
	i4I = 0;
      // hier nur zwischenspeichern
      SThetaS = RThetaR;
    }

#ifdef HAVE_AMVIS
    void setAMVisBody(AMVis::CRigidBody *body, CoordinateSystem* cosy=0, DataInterfaceBase* funcColor=0) {bodyAMVis=body; bodyAMVisUserFunctionColor=funcColor; cosyAMVis=(cosy==0)?port[0]:cosy;}
#endif

    void plot(double t, double dt=1);

    void addCoordinateSystem(CoordinateSystem *port_, const Vec &RrRK, const SqrMat &ARK, const CoordinateSystem* refCoordinateSystem=0); 

    void addCoordinateSystem(const string &str, const Vec &SrSK, const SqrMat &ASK, const CoordinateSystem* refCoordinateSystem=0);

    void addContour(Contour* contour, const Vec &RrRC, const SqrMat &ARC, const CoordinateSystem* refCoordinateSystem=0);
    
    void setReferenceCoordinateSystem(CoordinateSystem *port) {
      iRef = portIndex(port);
      assert(iRef > -1);
    }

    void setParentCoordinateSystem(CoordinateSystem *port) {portParent = port;};

    double computeKineticEnergy();
    double computeKineticEnergyBranch();
    double computePotentialEnergyBranch();

    void init();
    void initPlotFiles();
    void calcSize();

  };

  typedef RigidBody BodyRigid;

}

#endif

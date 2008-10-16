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
#include "coordinate_system.h"
#include "userfunction.h"
#include "kinematics.h"
#ifdef HAVE_AMVIS
namespace AMVis {class CRigidBody;}
#endif

// Projektive Newton-Euler-Gleichungen bzgl. COG


namespace MBSim {

  /*! \brief Class for rigid bodies with relative coordinates 
   *
   * */
  class RigidBody : public Body {

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
    void setInertiaTensor(const SymMat& RThetaR, const CoordinateSystem* refCoordinateSystem=0) {
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
    
    void setCoordinateSystemForKinematics(CoordinateSystem *port) {
      iRef = portIndex(port);
      assert(iRef > -1);
    }

    void setFrameOfReference(CoordinateSystem *port) {portParent = port;};

    double computeKineticEnergy();
    double computeKineticEnergyBranch();
    double computePotentialEnergyBranch();

    void init();
    void initPlotFiles();
    void calcSize();

    virtual string getType() const {return "RigidBody";}

    void load(const string &path, ifstream &inputfile);
    void save(const string &path, ofstream &outputfile);

  };

}

#endif

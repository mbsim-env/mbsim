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

#ifndef _BODY_RIGID_REL_H_
#define _BODY_RIGID_REL_H_

#include "body_rigid.h"
#include "fmatvec.h"
#include <vector>
#include "tree.h"

namespace MBSim {

  /*! \brief Class for rigid bodies with relative coordinates 
   *
   * */
  class BodyRigidRel : public BodyRigid {

    friend class Tree;

    protected:
    Index Il;
    int lSize, lInd;
    Tree *tree;
    vector<BodyRigidRel*> successor;
    BodyRigidRel* precessor;
    SqrMat APK, APK0;
    Vec PrPK, PrPK0, KrOK, KvK, e;
    Index IuT, IuR, iI;
    SqrMat C;

    virtual void updateKinematics(double t);
    virtual void updateCenterOfGravity(double t);
    virtual void updateh(double t);
    virtual void updateWj(double t);
    virtual void updater(double t);
    virtual void updatedq(double t, double dt);
    virtual void updateqd(double t);

    void updateM(double t);
    void updateMh(double t);

    public:

    BodyRigidRel(const string &name);

    void calcSize();
    void setlInd(int lInd_) { lInd = lInd_; Il = Index(lInd,lInd+lSize-1); }
    const Index& getlIndex() const { return Il;}

    const Index& getIuT() const {return IuT;}
    const Index& getIuR() const {return IuR;}

    void addChild(BodyRigidRel *body);

    void setPrPK0(const Vec& PrPK0_) {PrPK0 = PrPK0_;}
    void setAPK0(const SqrMat &APK0_) {APK0 = APK0_;}

    const Vec& getPrPK0() const {return PrPK0;}

    const Vec& getWrOK() const {return WrOK;}
    const Vec& getWvK() const {return WvK;}
    const Vec& getKrOK() const {return KrOK;}
    const Vec& getKvK() const {return KvK;}
    const Vec& gete() const {return e;}
    const Mat& getJ() const {return J;}

    //const Vec computeWvS()  const {return WvK  + crossProduct(KomegaK,WrKS);}


    double computeKineticEnergy();

    int getWSize() const { return tree->getuSize(); }
    void initStage1();
    void initStage2();
    void initz();
    void initPlotFiles();
    void plot(double t, double dt = 1);
    void setPrecessor(BodyRigidRel *precessor_);
    BodyRigidRel* getPrecessor() {return precessor;}
    void updateqRef();
    void updateqdRef();
    void updatezdRef();
    void updateuRef();
    void updatehRef();
    void updaterRef();
    void updateMhRef();
    void updateTRef();
    virtual void updatelRef();
    virtual void updateJRef();

    void setMbs(MultiBodySystem* mbs);
    void setTree(Tree *tree);
    void updateFullName();

    Port* getPort(const string &pName);
    Contour* getContour(const string &cName);
  };

}

#endif

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

#ifndef _TREE_H_
#define _TREE_H_

#include <string>
#include <vector>
#include "object.h"


using namespace std;

namespace MBSim {

  class BodyRigidRel;

  /*! \brief Class for subsystems with tree structure
   *
   * */
  class Tree : public Object {

    friend class BodyRigidRel;
    private:

    protected:
    int lSize;
    BodyRigidRel *root;
    Vec l;
    Mat J;
    SymMat Mh;
    Vec invMh;

    double computePotentialEnergyBranch(BodyRigidRel* body);

    public:

    Tree(const string &projectName);
    ~Tree();

    void updateKinematics(double t);
    void updateh(double t);
    void updateW(double t);
    void updatezd(double t);
    void updatedq(double t, double dt);
    void updatedu(double t, double dt);

    const Vec& getl() const {return l;}
    Vec& getl() {return l;}
    const Mat& getJ() const {return J;}
    Mat& getJ() {return J;}
    const SymMat& getMh() const {return Mh;}
    SymMat& getMh() {return Mh;}
    const Vec& getInvMh() const {return invMh;}
    Vec& getInvMh() {return invMh;}

    void calcSize();

    int getlSize() const { return lSize; }
    void setqSize(int qSize_) { qSize = qSize_; }
    void setuSize(int uSize_) { uSize = uSize_; }
    void setxSize(int xSize_) { xSize = xSize_; }
    void setlSize(int lSize_) { lSize = lSize_; }

    void setRoot(BodyRigidRel* root_);
    void updateqRef();
    void updateqdRef();
    void updatezdRef();
    void updateuRef();
    void updatehRef();
    void updaterRef();
    void init();
    void initz();
    void initPlotFiles();
    void plot(double t, double dt=1);

    void setMbs(MultiBodySystem* mbs);
    void setFullName(const string &name);

    double computePotentialEnergy();

  };

}

#endif

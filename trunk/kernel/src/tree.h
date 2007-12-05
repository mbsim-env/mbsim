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

  /*! \brief Class for subsystems with tree structure
   *
   * */
  class Tree : public Object {
    friend class BodyRigidRel;

    private:

    protected:
    int lSize;
    Vec l;
    Mat J;
    SymMat Mh;
    Vec invMh;

    public:

    Tree(const string &projectName);
    ~Tree();

    virtual void updateKinematics(double t) = 0;
    virtual void updateh(double t) = 0;
    virtual void updateM(double t) = 0;
    virtual void updateT(double t) = 0;
    virtual void updateWj(double t) = 0;
    virtual void updatezd(double t) = 0;
    virtual void updatedq(double t, double dt) = 0;
    virtual void updatedu(double t, double dt) = 0;

    virtual void calcSize() = 0;

    virtual void updateqRef() = 0;
    virtual void updateqdRef() = 0;
    virtual void updatezdRef() = 0;
    virtual void updateuRef() = 0;
    virtual void updatehRef() = 0;
    virtual void updaterRef() = 0;
    virtual void updateTRef() = 0;

    const Vec& getl() const {return l;}
    Vec& getl() {return l;}
    const Mat& getJ() const {return J;}
    Mat& getJ() {return J;}
    const SymMat& getMh() const {return Mh;}
    SymMat& getMh() {return Mh;}
    const Vec& getInvMh() const {return invMh;}
    Vec& getInvMh() {return invMh;}

    int getlSize()   const    { return lSize;   }
    void setqSize(int qSize_) { qSize = qSize_; }
    void setuSize(int uSize_) { uSize = uSize_; }
    void setxSize(int xSize_) { xSize = xSize_; }
    void setlSize(int lSize_) { lSize = lSize_; }

    virtual Port* getPort(const string &pName) = 0;
    virtual Contour* getContour(const string &cName) = 0;

    void addPort(Port * port_);	
    void addContour(Contour* contour_);
    //  tree::vector<Port*> port:    list of all ports of all single Bodies of the whole tree are temporary stored (the tree itself has no port)
    // 	(analog contour)	     so all ports can be accesed by the object-container of the multibodysystem (e.g. to create ports2plot-List)
    //				     Its only a temporary List; there is no data management (e.g. update, delete ..) within this class
    //				     (Data managment is done e.g. by BodyRigidRel)
  };

}

#endif

/* Copyright (C) 2007 Roland Zander
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
 * Contact:
 *    rzander@users.berlios.de
 */

#ifndef _TREE_FLEXROOT_H_
#define _TREE_FLEXROOT_H_

#include "tree.h"

namespace MBSim {
  class BodyFlexible;
  class BodyRigidRel;
  class BodyRigidRelOnFlex;

  /*! \brief class for tree-structured systems with flexible root body
 *
 * */

  class TreeFlexRoot : public Tree {
    friend class BodyFlexible;
    friend class BodyRigidRel;
    friend class BodyRigidRelOnFlex;

    protected:
    /** flexible root body */
    BodyFlexible *flexible;
    /** Index of flexible body parameters */
    Index Iflexible;
    /** Index of parameters of all rigid bodies*/
    Index Irigid;
    vector<BodyRigidRelOnFlex*> rigid;
//    vector<Index> Irigid;

    void initFlexibleStage1();
    void initFlexibleStage2();

    public:
    TreeFlexRoot(const string &name);
    ~TreeFlexRoot();

	Body* getRoot();

    void updateKinematics(double t);
    void updateh(double t);
    void updateM(double t);
    void updateT(double t);
    void updateWj(double t);
    void updatezd(double t);
//    void updateqd(double t);
    void updatedq(double t, double dt);
    void updatedu(double t, double dt);

    void calcSize();

    /*! */
    void setBodyFlexible(BodyFlexible* flexible_);
    /*! */
    void addBodyRigidRelOnFlex(BodyRigidRelOnFlex* body_,Vec s0_ = 0);

    void updateMRef();
    void updateqRef();
    void updateqdRef();
    void updatezRef();
    void updatezdRef();
    void updateuRef();
    void updatehRef();
    void updaterRef();
    void updateTRef();
    void init();
    void initz();
    void initPlotFiles();
    void plot(double t, double dt=1);
	void plotParameters();
	void plotParameterFiles();

    void setMbs(MultiBodySystem* mbs);
    void setFullName(const string &name);

    double computePotentialEnergy();
//    double computeKineticEnergy();
   };

}
#endif

/* Copyright (C) 2004-2006  Roland Zander
 
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

#ifndef _BODY_RIGID_REL_ONFLEX_H_
#define _BODY_RIGID_REL_ONFLEX_H_

#include "body_rigid_rel.h"
#include "fmatvec.h"
#include <vector>
#include "contour_pdata.h"

namespace MBSim {
  class BodyFlexible;

  /*! \brief Class for rigid bodies with relative coordinates 
   *
   * */
  class BodyRigidRelOnFlex : public BodyRigidRel {

    friend class Tree;
    friend class TreeFlexRoot;

    protected:

    BodyFlexible* precessor;
    SqrMat AWP;
//    Vec s0;
    bool constcPosition;
    ContourPointData cPosition;
    Index IJactive;

    void updateCenterOfGravity(double t);
    void updateh(double t);

//    void updateM(double t);

    public:
    BodyRigidRelOnFlex(const string &name);

    const Index& getIJactive() const {return IJactive;}
    void sets0(const Vec& s0_);

//    void plot(double t, double dt = 1);
    void initStage1();

    void setPrecessor(BodyFlexible *precessor_) {precessor=precessor_;}
    BodyFlexible* getPrecessor() {return precessor;}

    double computePotentialEnergyBranch();
  };

}

#endif

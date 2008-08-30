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

#ifndef _TREE_H_
#define _TREE_H_

#include "subsystem.h"


using namespace std;


namespace MBSim {
class Object;

  /*! \brief class for tree-structured systems with only rigid bodies
   *
   * */
  class Tree : public Subsystem {
    friend class Object;

    protected:

    //double computePotentialEnergyBranch(Object* body);

    public:

    Tree(const string &projectName);
    ~Tree();

    void updateKinematics(double t);
    void updateT(double t);
    void updateh(double t);
    void updateM(double t);

    void calchSize();

    //void setMbs(MultiBodySystem* mbs_);
    void setFullName(const string &name);

    void updateMRef();
    void updateLLMRef();
    void updateqRef();
    void updateqdRef();
    void updatezdRef();
    void updateuRef();
    void updatehRef();
    void updaterRef();
    void updateTRef();

    double computePotentialEnergy();

  };

}

#endif

/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/constraints/constraint.h"
#include "mbsim/dynamic_system_solver.h"
#include <hdf5serie/simpledataset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  Constraint::Constraint(const std::string &name) : Element(name), updGC(true), updGJ(true) {
  }

  void Constraint::updatexRef(const Vec &xParent) {
    x >> xParent(xInd,xInd+xSize-1);
  } 

  void Constraint::updatexdRef(const Vec &xdParent) {
    xd >> xdParent(xInd,xInd+xSize-1);
  } 

  void Constraint::updatedxRef(const Vec &dxParent) {
    dx >> dxParent(xInd,xInd+xSize-1);
  }

  void Constraint::initz() {
    x = (x0.size()==0)? Vec(xSize, INIT, 0) : x0;
  }

  void Constraint::writez(H5::GroupBase *group) {
    group->createChildObject<H5::SimpleDataset<vector<double> > >("x0")(x.size())->write(x);
  }

  void Constraint::readz0(H5::GroupBase *group) {
    x0.resize() = group->openChildObject<H5::SimpleDataset<vector<double> > >("x0")->read();
  }

  const Vec& Constraint::evalxd() {
    if(ds->getUpdatezd()) ds->updatezd();
    return xd;
  }

}

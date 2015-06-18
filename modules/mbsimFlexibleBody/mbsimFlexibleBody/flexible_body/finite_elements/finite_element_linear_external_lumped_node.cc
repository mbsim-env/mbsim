/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 *          rzander@users.berlios.de
 */


#include <config.h>
#include <iostream>
#include <finite_element_linear_external_lumped_node.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;




namespace MBSimFlexibleBody {
  FiniteElementLinearExternalLumpedNode::FiniteElementLinearExternalLumpedNode(double& mij_, fmatvec::Vec3& u0_, const fmatvec::Mat3xV& phi_) : DiscretizationInterface(), mij(mij_),u0(u0_), phi(phi_) {
//    cout << "From lumpedNode mij =" << mij<< endl;
//    cout << "From lumpedNode u0 =" << u0<< endl;
//    cout << "From lumpedNode phi =" << phi<< endl;

  }

  FiniteElementLinearExternalLumpedNode::~FiniteElementLinearExternalLumpedNode(){

  }

}

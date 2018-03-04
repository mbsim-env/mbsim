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
  FiniteElementLinearExternalLumpedNode::FiniteElementLinearExternalLumpedNode(double& mij_, Vec3& u0_, const Mat3xV& phi_) :  mij(mij_),u0(u0_), phi(phi_) {
//    msg(Debug) << "From lumpedNode mij =" << mij<< endl;
//    msg(Debug) << "From lumpedNode u0 =" << u0<< endl;
//    msg(Debug) << "From lumpedNode phi =" << phi<< endl;
  }

  FiniteElementLinearExternalLumpedNode::~FiniteElementLinearExternalLumpedNode()= default;

  const SymMat& FiniteElementLinearExternalLumpedNode::getM() const {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getM): Not implemented");
  }

  const Vec& FiniteElementLinearExternalLumpedNode::geth() const {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::geth): Not implemented");
  }

  const SqrMat& FiniteElementLinearExternalLumpedNode::getdhdq() const {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getdhdq): Not implemented");
  }

  const SqrMat& FiniteElementLinearExternalLumpedNode::getdhdu() const {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getdhdu): Not implemented");
  }

  int FiniteElementLinearExternalLumpedNode::getqSize() const { throw runtime_error("(FiniteElementLinearExternalLumpedNode::getqSize): Not implemented"); }

  int FiniteElementLinearExternalLumpedNode::getuSize() const {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getuSize): Not implemented");
  }

  void  FiniteElementLinearExternalLumpedNode::computeM(const Vec& qG) { throw runtime_error("(FiniteElementLinearExternalLumpedNode::computeM): Not implemented"); }

  void  FiniteElementLinearExternalLumpedNode::computeh(const Vec& qG, const Vec& qGt) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::computeh): Not implemented");
  }

  void  FiniteElementLinearExternalLumpedNode::computedhdz(const Vec& qG, const Vec& qGt) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::computedhdz): Not implemented");
  }

  double FiniteElementLinearExternalLumpedNode::computeKineticEnergy(const Vec& qG, const Vec& qGt) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::computeKineticEnergy): Not implemented");
  }

  double FiniteElementLinearExternalLumpedNode::computeGravitationalEnergy(const Vec& qG) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::computeGravitationalEnergy): Not implemented");
  }

  double FiniteElementLinearExternalLumpedNode::computeElasticEnergy(const Vec& qG) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::computeElasticEnergy): Not implemented");
  }

  Vec3 FiniteElementLinearExternalLumpedNode::getPosition(const Vec& qElement, double s) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getPosition): Not implemented!");
  }

  SqrMat3 FiniteElementLinearExternalLumpedNode::getOrientation(const Vec& qElement, double s) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getOrientation): Not implemented!");
  }

  Vec3 FiniteElementLinearExternalLumpedNode::getVelocity(const Vec& qElement, const Vec& qpElement, double s) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getVelocity): Not implemented!");
  }

  Vec3 FiniteElementLinearExternalLumpedNode::getAngularVelocity(const Vec& qElement, const Vec& qpElement, double s) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getAngularVelocity): Not implemented!");
  }

  Mat FiniteElementLinearExternalLumpedNode::getJacobianOfMotion(const Vec& qElement, double s) {
    throw runtime_error("(FiniteElementLinearExternalLumpedNode::getJacobianOfMotion): Not implemented");
  }

}

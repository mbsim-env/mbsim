/* Copyright (C) 2005-2008  Roland Zander

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

#ifndef _BODY_FLEXIBLE_1S_23_BTA_H_
#define _BODY_FLEXIBLE_1S_23_BTA_H_

#include "body_flexible.h"
#include "contour.h"

namespace MBSim {

  /*! \brief bending torsional axis
   * \todo gravity, handling for Contour-node information
   */
  class BodyFlexible1s23BTA : public BodyFlexible1s {

	public:
	  BodyFlexible1s23BTA(const string &name); 
	  virtual ~BodyFlexible1s23BTA() {}

	  void setNumberElements(int n); 
	  void setLength(double L_) {L = L_;}
	  void setElastModuls(double E_, double G_) {E = E_;G = G_;}
	  void setCrossSectionalArea(double A_) {A = A_;}
	  void setMomentsInertia(double Iyy_,double Izz_,double It_) {Iyy = Iyy_;Izz = Izz_;It = It_;}
	  void setDensity(double rho_) {rho = rho_;}
	  void setTorsionalDamping(double d) {dTorsional = d;}
	  void setContourRadius(double r) {contourCyl->setRadius(r);}

	  using BodyFlexible1s::addPort;
	  void addPort(const string &name, const int &node);

	  Mat computeJacobianMatrix(const ContourPointData &data);

	  Vec computeWn(const ContourPointData &S_){return computeAWK(S_).col(1);}
	  Mat computeWt(const ContourPointData &S_);
	  Vec computeWrOC(const ContourPointData &S_);
	  Vec computeWvC(const ContourPointData &S_);
	  Vec computeWomega(const ContourPointData &S_);
	  SqrMat computeAWK(const ContourPointData &S_);

	  bool hasConstMass() const {return false;}

	  void setJ(const Mat &J_) {JR =J_;  JT = J_(0,1,2,2);}

	  void setWrON00(const Vec &WrON00_) {WrON00 = WrON00_;}

	  void plotParameters();

	protected:
	  int Elements;
	  double L, l0, E, G, A, Iyy, Izz, It,rho, rc, dTorsional;

	  bool implicit;
	  SqrMat Dhq, Dhqp;

	  Vec WrON00;

	  void BuildElements();
	  double BuildElement(const double&);

	  void updateKinematics(double t);
	  void updatePorts(double t);

	  void GlobalMatrixContribution(int n);

	  void init();

	  double sTangent;
	  Vec Wt, WrOC, WvC, Womega;

	  CylinderFlexible *contourCyl;
	
	private:
	  int CurrentElement;
	  Index activeElement;
  };

}

#endif

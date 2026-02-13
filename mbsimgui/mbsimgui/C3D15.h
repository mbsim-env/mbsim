/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2022 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _C3D15_H_
#define _C3D15_H_

#include "fe_type.h"

namespace MBSimGUI {

  class C3D15 : public FiniteElementType {
    public:
      C3D15();

      double N(int i, double x, double y, double z) const override {
	return (this->*N_[i])(x,y,z);
      }

      double dNdq(int i, int j, double x, double y, double z) const override {
	return (this->*dNdq_[i][j])(x,y,z);
      }

      double NA(int i, double x, double y) const override {
        std::runtime_error("Distributed loads not yet implemented for C3D15."); return 0;
      }

      double dNAdq(int i, int j, double x, double y) const override {
        std::runtime_error("Distributed loads not yet implemented for C3D15."); return 0;
      }

      int getNumberOfNodes() const override { return 15; }
      int getNumberOf2DNodes(int face) const override { return face<2?6:8; }
      int getNumberOfIntegrationPoints() const override { return 9; }
      int getNumberOf2DIntegrationPoints(int face) const override { return face<2?3:4; }
      int getNumberOfExtrapolationPoints() const override { return 6; }
      const fmatvec::Vec3& getNaturalCoordinates(int i) const override { return rN[i]; }
      const fmatvec::Vec3& getIntegrationPoint(int i) const override { return rI[i]; }
      const fmatvec::Vec2& get2DIntegrationPoint(int face, int i) const override { return face<2?rAI[0][i]:rAI[1][i]; }
      double getWeight(int i) const override { return wI[i]; }
      double get2DWeight(int face, int i) const override { return face<2?wAI[0][i]:wAI[1][i]; }
      double getExtrapolationCoefficient(int i, int j) const override { return A[i][j]; }
      int getExtrapolationIndex(int i, int j) const override { return B[i][j]; }
      int getNumberOfFaces() const override { return 5; }
      int getNodeNumberOnFace(int i, int j) const { return fI[i][j]; }

    private:
      fmatvec::Vec3 rN[15], rI[9];
      fmatvec::Vec2 rAI[2][4];
      double wI[9], wAI[2][4];
      double A[9][6];
      int B[9][2];
      double (C3D15::*N_[15])(double x, double y, double z) const;
      double (C3D15::*dNdq_[15][3])(double x, double y, double z) const;
      int fI[5][8];

      double N1(double x, double y, double z) const {
	return 0.5*((1-x-y)*(2*(1-x-y)-1)*(1-z)-(1-x-y)*(1-z*z));
      }

      double N2(double x, double y, double z) const {
	return 0.5*(x*(2*x-1)*(1-z)-x*(1-z*z));
      }

      double N3(double x, double y, double z) const {
	return 0.5*(y*(2*y-1)*(1-z)-y*(1-z*z));
      }

      double N4(double x, double y, double z) const {
	return 0.5*((1-x-y)*(2*(1-x-y)-1)*(1+z)-(1-x-y)*(1-z*z));
      }

      double N5(double x, double y, double z) const {
	return 0.5*(x*(2*x-1)*(1+z)-x*(1-z*z));
      }

      double N6(double x, double y, double z) const {
	return 0.5*(y*(2*y-1)*(1+z)-y*(1-z*z));
      }

      double N7(double x, double y, double z) const {
	return 2*(1-x-y)*x*(1-z);
      }

      double N8(double x, double y, double z) const {
	return 2*x*y*(1-z);
      }

      double N9(double x, double y, double z) const {
	return 2*y*(1-x-y)*(1-z);
      }

      double N10(double x, double y, double z) const {
	return 2*(1-x-y)*x*(1+z);
      }

      double N11(double x, double y, double z) const {
	return 2*x*y*(1+z);
      }

      double N12(double x, double y, double z) const {
	return 2*y*(1-x-y)*(1+z);
      }

      double N13(double x, double y, double z) const {
	return (1-x-y)*(1-z*z);
      }

      double N14(double x, double y, double z) const {
	return x*(1-z*z);
      }

      double N15(double x, double y, double z) const {
	return y*(1-z*z);
      }

      double dN1dx(double x, double y, double z) const {
	return 0.5*((1-4*(1-x-y))*(1-z)+(1-z*z));
      }

      double dN1dy(double x, double y, double z) const {
	return 0.5*((1-4*(1-x-y))*(1-z)+(1-z*z));
      }

      double dN1dz(double x, double y, double z) const {
	return (1-x-y)*z-0.5*(1-x-y)*(2*(1-x-y)-1);
      }

      double dN2dx(double x, double y, double z) const {
	return 0.5*((4*x-1)*(1-z)-(1-z*z));
      }

      double dN2dy(double x, double y, double z) const {
	return 0;
      }

      double dN2dz(double x, double y, double z) const {
	return x*z-0.5*x*(2*x-1);
      }

      double dN3dx(double x, double y, double z) const {
	return 0;
      }

      double dN3dy(double x, double y, double z) const {
	return 0.5*((4*y-1)*(1-z)-(1-z*z));
      }

      double dN3dz(double x, double y, double z) const {
	return y*z-0.5*y*(2*y-1);
      }

      double dN4dx(double x, double y, double z) const {
	return 0.5*((1-4*(1-x-y))*(1+z)+(1-z*z));
      }

      double dN4dy(double x, double y, double z) const {
	return 0.5*((1-4*(1-x-y))*(1+z)+(1-z*z)); 
      }

      double dN4dz(double x, double y, double z) const {
	return (1-x-y)*z+0.5*(1-x-y)*(2*(1-x-y)-1);
      }

      double dN5dx(double x, double y, double z) const {
	return 0.5*((4*x-1)*(1+z)-(1-z*z));
      }

      double dN5dy(double x, double y, double z) const {
	return 0;
      }

      double dN5dz(double x, double y, double z) const {
	return x*z+0.5*x*(2*x-1);
      }

      double dN6dx(double x, double y, double z) const {
	return 0;
      }

      double dN6dy(double x, double y, double z) const {
	return 0.5*((4*y-1)*(1+z)-(1-z*z));
      }

      double dN6dz(double x, double y, double z) const {
	return y*z+0.5*y*(2*y-1);
      }

      double dN7dx(double x, double y, double z) const {
	return 2*(1-x-y-x)*(1-z);
      }

      double dN7dy(double x, double y, double z) const {
	return -2*x*(1-z);
      }

      double dN7dz(double x, double y, double z) const {
	return -2*(1-x-y)*x;
      }

      double dN8dx(double x, double y, double z) const {
	return 2*y*(1-z);
      }

      double dN8dy(double x, double y, double z) const {
	return 2*x*(1-z);
      }

      double dN8dz(double x, double y, double z) const {
	return -2*x*y;
      }

      double dN9dx(double x, double y, double z) const {
	return -2*y*(1-z);
      }

      double dN9dy(double x, double y, double z) const {
	return 2*(1-x-y-y)*(1-z);
      }

      double dN9dz(double x, double y, double z) const {
	return -2*y*(1-x-y);
      }

      double dN10dx(double x, double y, double z) const {
	return 2*(1-x-y-x)*(1+z);
      }

      double dN10dy(double x, double y, double z) const {
	return -2*x*(1+z);
      }

      double dN10dz(double x, double y, double z) const {
	return 2*(1-x-y)*x;
      }

      double dN11dx(double x, double y, double z) const {
	return 2*y*(1+z);
      }

      double dN11dy(double x, double y, double z) const {
	return 2*x*(1+z);
      }

      double dN11dz(double x, double y, double z) const {
	return 2*x*y;
      }

      double dN12dx(double x, double y, double z) const {
	return -2*y*(1+z);
      }

      double dN12dy(double x, double y, double z) const {
	return 2*(1-x-y-y)*(1+z);
      }

      double dN12dz(double x, double y, double z) const {
	return 2*y*(1-x-y);
      }

      double dN13dx(double x, double y, double z) const {
	return -(1-z*z);
      }

      double dN13dy(double x, double y, double z) const {
	return -(1-z*z);
      }

      double dN13dz(double x, double y, double z) const {
	return -(1-x-y)*2*z;
      }

      double dN14dx(double x, double y, double z) const {
	return (1-z*z);
      }

      double dN14dy(double x, double y, double z) const {
	return 0;
      }

      double dN14dz(double x, double y, double z) const {
	return -2*x*z;
      }

      double dN15dx(double x, double y, double z) const {
	return 0;
      }

      double dN15dy(double x, double y, double z) const {
	return (1-z*z);
      }

      double dN15dz(double x, double y, double z) const {
	return -2*y*z;
      }

  };

}

#endif

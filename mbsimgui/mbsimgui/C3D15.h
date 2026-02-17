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

      double NA(int face, int i, double x, double y) const override {
	return face<2?(this->*NA_[0][i])(x,y):(this->*NA_[1][i])(x,y);
      }

      double dNAdq(int face, int i, int j, double x, double y) const override {
	return face<2?(this->*dNAdq_[0][i][j])(x,y):(this->*dNAdq_[1][i][j])(x,y);
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
      double (C3D15::*NA_[2][8])(double x, double y) const;
      double (C3D15::*dNAdq_[2][8][2])(double x, double y) const;
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

      double NA1a(double x, double y) const {
        return 2*(0.5-x-y)*(1-x-y);
      }

      double NA2a(double x, double y) const {
        return x*(2*x-1);
      }

      double NA3a(double x, double y) const {
        return y*(2*y-1);
      }

      double NA4a(double x, double y) const {
        return 4*x*(1-x-y);
      }

      double NA5a(double x, double y) const {
        return 4*x*y;
      }

      double NA6a(double x, double y) const {
        return 4*y*(1-x-y);
      }

      double dNA1dxa(double x, double y) const {
        return 4*(x+y)-3;
      }

      double dNA1dya(double x, double y) const {
        return 4*(x+y)-3;
      }

      double dNA2dxa(double x, double y) const {
        return 4*x-1;
      }

      double dNA2dya(double x, double y) const {
        return 0;
      }

      double dNA3dxa(double x, double y) const {
        return 0;
      }

      double dNA3dya(double x, double y) const {
        return 4*y-1;
      }

      double dNA4dxa(double x, double y) const {
        return 4*(1-x-y-x);
      }

      double dNA4dya(double x, double y) const {
        return -4*x;
      }

      double dNA5dxa(double x, double y) const {
        return 4*y;
      }

      double dNA5dya(double x, double y) const {
        return 4*x;
      }

      double dNA6dxa(double x, double y) const {
        return -4*y;
      }

      double dNA6dya(double x, double y) const {
        return 4*(1-x-y-y);
      }

      double NA1b(double x, double y) const {
        return 1./4*(1-x)*(1-y)*(-x-y-1);
      }
      double NA2b(double x, double y) const {
        return 1./4*(1+x)*(1-y)*(x-y-1);
      }
      double NA3b(double x, double y) const {
        return 1./4*(1+x)*(1+y)*(x+y-1);
      }
      double NA4b(double x, double y) const {
        return 1./4*(1-x)*(1+y)*(-x+y-1);
      }
      double NA5b(double x, double y) const {
        return 1./2*(1+x)*(1-x)*(1-y);
      }
      double NA6b(double x, double y) const {
        return 1./2*(1+x)*(1+y)*(1-y);
      }
      double NA7b(double x, double y) const {
        return 1./2*(1+x)*(1-x)*(1+y);
      }
      double NA8b(double x, double y) const {
        return 1./2*(1-x)*(1+y)*(1-y);
      }
      double dNA1dxb(double x, double y) const {
        return 1./4*(1-y)*(2*x+y);
      }
      double dNA1dyb(double x, double y) const {
        return 1./4*(1-x)*(x+2*y);
      }
      double dNA2dxb(double x, double y) const {
        return 1./4*(1-y)*(2*x-y);
      }
      double dNA2dyb(double x, double y) const {
        return 1./4*(1+x)*(-x+2*y);
      }
      double dNA3dxb(double x, double y) const {
        return 1./4*(1+y)*(2*x+y);
      }
      double dNA3dyb(double x, double y) const {
        return 1./4*(1+x)*(x+2*y);
      }
      double dNA4dxb(double x, double y) const {
        return 1./4*(1+y)*(2*x-y);
      }
      double dNA4dyb(double x, double y) const {
        return 1./4*(1-x)*(-x+2*y);
      }
      double dNA5dxb(double x, double y) const {
        return -(1-y)*x;
      }
      double dNA5dyb(double x, double y) const {
        return -1./2*(1+x)*(1-x);
      }
      double dNA6dxb(double x, double y) const {
        return 1./2*(1+y)*(1-y);
      }
      double dNA6dyb(double x, double y) const {
        return -(1+x)*y;
      }
      double dNA7dxb(double x, double y) const {
        return -(1+y)*x;
      }
      double dNA7dyb(double x, double y) const {
        return 1./2*(1+x)*(1-x);
      }
      double dNA8dxb(double x, double y) const {
        return -1./2*(1+y)*(1-y);
      }
      double dNA8dyb(double x, double y) const {
        return -(1-x)*y;
      }

  };

}

#endif

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

#ifndef _C3D20BASE_H_
#define _C3D20BASE_H_

#include "fe_type.h"

namespace MBSimGUI {

  class C3D20Base : public FiniteElementType {
    public:
      C3D20Base();

      double N(int i, double x, double y, double z) const override {
	return (this->*N_[i])(x,y,z);
      }

      double dNdq(int i, int j, double x, double y, double z) const override {
	return (this->*dNdq_[i][j])(x,y,z);
      }

      double NA(int face, int i, double x, double y) const override {
	return (this->*NA_[i])(x,y);
      }

      double dNAdq(int face, int i, int j, double x, double y) const override {
	return (this->*dNAdq_[i][j])(x,y);
      }

      int getNumberOfNodes() const override { return 20; }
      int getNumberOf2DNodes(int face) const override { return 8; }
      const fmatvec::Vec3& getNaturalCoordinates(int i) const override { return rN[i]; }
      int getNumberOfFaces() const override { return 6; }
      int getNodeNumberOnFace(int i, int j) const { return fI[i][j]; }

    private:
      fmatvec::Vec3 rN[20];
      double (C3D20Base::*N_[20])(double x, double y, double z) const;
      double (C3D20Base::*dNdq_[20][3])(double x, double y, double z) const;
      double (C3D20Base::*NA_[8])(double x, double y) const;
      double (C3D20Base::*dNAdq_[8][2])(double x, double y) const;
      int fI[6][8];

      double N1(double x, double y, double z) const {
	return 1./8*(1-x)*(1-y)*(1-z)*(-x-y-z-2);
      }
      double N2(double x, double y, double z) const {
	return 1./8*(1+x)*(1-y)*(1-z)*(x-y-z-2);
      }
      double N3(double x, double y, double z) const {
	return 1./8*(1+x)*(1+y)*(1-z)*(x+y-z-2);
      }
      double N4(double x, double y, double z) const {
	return 1./8*(1-x)*(1+y)*(1-z)*(-x+y-z-2);
      }
      double N5(double x, double y, double z) const {
	return 1./8*(1-x)*(1-y)*(1+z)*(-x-y+z-2);
      }
      double N6(double x, double y, double z) const {
	return 1./8*(1+x)*(1-y)*(1+z)*(x-y+z-2);
      }
      double N7(double x, double y, double z) const {
	return 1./8*(1+x)*(1+y)*(1+z)*(x+y+z-2);
      }
      double N8(double x, double y, double z) const {
	return 1./8*(1-x)*(1+y)*(1+z)*(-x+y+z-2);
      }
      double N9(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1-y)*(1-z);
      }
      double N10(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1-z)*(1+x);
      }
      double N11(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1+y)*(1-z);
      }
      double N12(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1-z)*(1-x);
      }
      double N13(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1-y)*(1+z);
      }
      double N14(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1+z)*(1+x);
      }
      double N15(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1+y)*(1+z);
      }
      double N16(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1+z)*(1-x);
      }
      double N17(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1-x)*(1-y);
      }
      double N18(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1+x)*(1-y);
      }
      double N19(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1+x)*(1+y);
      }
      double N20(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1-x)*(1+y);
      }
      double dN1dx(double x, double y, double z) const {
	return -1./8*(1-y)*(1-z)*(-2*x-y-z-1);
      }
      double dN1dy(double x, double y, double z) const {
	return -1./8*(1-z)*(1-x)*(-2*y-x-z-1);
      }
      double dN1dz(double x, double y, double z) const {
	return -1./8*(1-x)*(1-y)*(-2*z-x-y-1);
      }
      double dN2dx(double x, double y, double z) const {
	return 1./8*(1-y)*(1-z)*(2*x-y-z-1);
      }
      double dN2dy(double x, double y, double z) const {
	return -1./8*(1-z)*(1+x)*(-2*y+x-z-1);
      }
      double dN2dz(double x, double y, double z) const {
	return -1./8*(1+x)*(1-y)*(-2*z+x-y-1);
      }
      double dN3dx(double x, double y, double z) const {
	return 1./8*(1+y)*(1-z)*(2*x+y-z-1);
      }
      double dN3dy(double x, double y, double z) const {
	return 1./8*(1-z)*(1+x)*(2*y+x-z-1);
      }
      double dN3dz(double x, double y, double z) const {
	return -1./8*(1+x)*(1+y)*(-2*z+x+y-1);
      }
      double dN4dx(double x, double y, double z) const {
	return -1./8*(1+y)*(1-z)*(-2*x+y-z-1);
      }
      double dN4dy(double x, double y, double z) const {
	return 1./8*(1-z)*(1-x)*(2*y-x-z-1);
      }
      double dN4dz(double x, double y, double z) const {
	return -1./8*(1-x)*(1+y)*(-2*z-x+y-1);
      }
      double dN5dx(double x, double y, double z) const {
	return -1./8*(1-y)*(1+z)*(-2*x-y+z-1);
      }
      double dN5dy(double x, double y, double z) const {
	return -1./8*(1+z)*(1-x)*(-2*y-x+z-1);
      }
      double dN5dz(double x, double y, double z) const {
	return 1./8*(1-x)*(1-y)*(2*z-x-y-1);
      }
      double dN6dx(double x, double y, double z) const {
	return 1./8*(1-y)*(1+z)*(2*x-y+z-1);
      }
      double dN6dy(double x, double y, double z) const {
	return -1./8*(1+z)*(1+x)*(-2*y+x+z-1);
      }
      double dN6dz(double x, double y, double z) const {
	return 1./8*(1+x)*(1-y)*(2*z+x-y-1);
      }
      double dN7dx(double x, double y, double z) const {
	return 1./8*(1+y)*(1+z)*(2*x+y+z-1);
      }
      double dN7dy(double x, double y, double z) const {
	return 1./8*(1+z)*(1+x)*(2*y+x+z-1);
      }
      double dN7dz(double x, double y, double z) const {
	return 1./8*(1+x)*(1+y)*(2*z+x+y-1);
      }
      double dN8dx(double x, double y, double z) const {
	return -1./8*(1+y)*(1+z)*(-2*x+y+z-1);
      }
      double dN8dy(double x, double y, double z) const {
	return 1./8*(1+z)*(1-x)*(2*y-x+z-1);
      }
      double dN8dz(double x, double y, double z) const {
	return 1./8*(1-x)*(1+y)*(2*z-x+y-1);
      }
      double dN9dx(double x, double y, double z) const {
	return -1./2*x*(1-y)*(1-z);
      }
      double dN9dy(double x, double y, double z) const {
	return -1./4*(1-x*x)*(1-z);
      }
      double dN9dz(double x, double y, double z) const {
	return -1./4*(1-x*x)*(1-y);
      }
      double dN10dx(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1-z);
      }
      double dN10dy(double x, double y, double z) const {
	return -1./2*y*(1-z)*(1+x);
      }
      double dN10dz(double x, double y, double z) const {
	return -1./4*(1-y*y)*(1+x);
      }
      double dN11dx(double x, double y, double z) const {
	return -1./2*x*(1+y)*(1-z);
      }
      double dN11dy(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1-z);
      }
      double dN11dz(double x, double y, double z) const {
	return -1./4*(1-x*x)*(1+y);
      }
      double dN12dx(double x, double y, double z) const {
	return -1./4*(1-y*y)*(1-z);
      }
      double dN12dy(double x, double y, double z) const {
	return -1./2*y*(1-z)*(1-x);
      }
      double dN12dz(double x, double y, double z) const {
	return -1./4*(1-y*y)*(1-x);
      }
      double dN13dx(double x, double y, double z) const {
	return -1./2*x*(1-y)*(1+z);
      }
      double dN13dy(double x, double y, double z) const {
	return -1./4*(1-x*x)*(1+z);
      }
      double dN13dz(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1-y);
      }
      double dN14dx(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1+z);
      }
      double dN14dy(double x, double y, double z) const {
	return -1./2*y*(1+z)*(1+x);
      }
      double dN14dz(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1+x);
      }
      double dN15dx(double x, double y, double z) const {
	return -1./2*x*(1+y)*(1+z);
      }
      double dN15dy(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1+z);
      }
      double dN15dz(double x, double y, double z) const {
	return 1./4*(1-x*x)*(1+y);
      }
      double dN16dx(double x, double y, double z) const {
	return -1./4*(1-y*y)*(1+z);
      }
      double dN16dy(double x, double y, double z) const {
	return -1./2*y*(1+z)*(1-x);
      }
      double dN16dz(double x, double y, double z) const {
	return 1./4*(1-y*y)*(1-x);
      }
      double dN17dx(double x, double y, double z) const {
	return -1./4*(1-z*z)*(1-y);
      }
      double dN17dy(double x, double y, double z) const {
	return -1./4*(1-z*z)*(1-x);
      }
      double dN17dz(double x, double y, double z) const {
	return -1./2*z*(1-x)*(1-y);
      }
      double dN18dx(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1-y);
      }
      double dN18dy(double x, double y, double z) const {
	return -1./4*(1-z*z)*(1+x);
      }
      double dN18dz(double x, double y, double z) const {
	return -1./2*z*(1+x)*(1-y);
      }
      double dN19dx(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1+y);
      }
      double dN19dy(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1+x);
      }
      double dN19dz(double x, double y, double z) const {
	return -1./2*z*(1+x)*(1+y);
      }
      double dN20dx(double x, double y, double z) const {
	return -1./4*(1-z*z)*(1+y);
      }
      double dN20dy(double x, double y, double z) const {
	return 1./4*(1-z*z)*(1-x);
      }
      double dN20dz(double x, double y, double z) const {
	return -1./2*z*(1-x)*(1+y);
      }

      double NA1(double x, double y) const {
        return 1./4*(1-x)*(1-y)*(-x-y-1);
      }
      double NA2(double x, double y) const {
        return 1./4*(1+x)*(1-y)*(x-y-1);
      }
      double NA3(double x, double y) const {
        return 1./4*(1+x)*(1+y)*(x+y-1);
      }
      double NA4(double x, double y) const {
        return 1./4*(1-x)*(1+y)*(-x+y-1);
      }
      double NA5(double x, double y) const {
        return 1./2*(1+x)*(1-x)*(1-y);
      }
      double NA6(double x, double y) const {
        return 1./2*(1+x)*(1+y)*(1-y);
      }
      double NA7(double x, double y) const {
        return 1./2*(1+x)*(1-x)*(1+y);
      }
      double NA8(double x, double y) const {
        return 1./2*(1-x)*(1+y)*(1-y);
      }
      double dNA1dx(double x, double y) const {
        return 1./4*(1-y)*(2*x+y);
      }
      double dNA1dy(double x, double y) const {
        return 1./4*(1-x)*(x+2*y);
      }
      double dNA2dx(double x, double y) const {
        return 1./4*(1-y)*(2*x-y);
      }
      double dNA2dy(double x, double y) const {
        return 1./4*(1+x)*(-x+2*y);
      }
      double dNA3dx(double x, double y) const {
        return 1./4*(1+y)*(2*x+y);
      }
      double dNA3dy(double x, double y) const {
        return 1./4*(1+x)*(x+2*y);
      }
      double dNA4dx(double x, double y) const {
        return 1./4*(1+y)*(2*x-y);
      }
      double dNA4dy(double x, double y) const {
        return 1./4*(1-x)*(-x+2*y);
      }
      double dNA5dx(double x, double y) const {
        return -(1-y)*x;
      }
      double dNA5dy(double x, double y) const {
        return -1./2*(1+x)*(1-x);
      }
      double dNA6dx(double x, double y) const {
        return 1./2*(1+y)*(1-y);
      }
      double dNA6dy(double x, double y) const {
        return -(1+x)*y;
      }
      double dNA7dx(double x, double y) const {
        return -(1+y)*x;
      }
      double dNA7dy(double x, double y) const {
        return 1./2*(1+x)*(1-x);
      }
      double dNA8dx(double x, double y) const {
        return -1./2*(1+y)*(1-y);
      }
      double dNA8dy(double x, double y) const {
        return -(1-x)*y;
      }

  };

}

#endif

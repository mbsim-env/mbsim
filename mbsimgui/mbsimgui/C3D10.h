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

#ifndef _C3D10_H_
#define _C3D10_H_

#include "fe_type.h"

namespace MBSimGUI {

  class C3D10 : public FiniteElementType {
    public:
      C3D10();

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

      int getNumberOfNodes() const override { return 10; }
      int getNumberOf2DNodes(int face) const override { return 6; }
      int getNumberOfIntegrationPoints() const override { return 4; }
      int getNumberOf2DIntegrationPoints(int face) const override { return 3; }
      int getNumberOfExtrapolationPoints() const override { return 4; }
      const fmatvec::Vec3& getNaturalCoordinates(int i) const override { return rN[i]; }
      const fmatvec::Vec3& getIntegrationPoint(int i) const override { return rI[i]; }
      const fmatvec::Vec2& get2DIntegrationPoint(int face, int i) const override { return rAI[i]; }
      double getWeight(int i) const override { return wI[i]; }
      double get2DWeight(int face, int i) const override { return wAI[i]; }
      double getExtrapolationCoefficient(int i, int j) const override { return A[i][j]; }
      int getExtrapolationIndex(int i, int j) const override { return B[i][j]; }
      int getNumberOfFaces() const override { return 4; }
      int getNodeNumberOnFace(int i, int j) const { return fI[i][j]; }

    private:
      fmatvec::Vec3 rN[10], rI[4];
      fmatvec::Vec2 rAI[3];
      double wI[4], wAI[3];
      double A[4][4];
      int B[6][2];
      double (C3D10::*N_[10])(double x, double y, double z) const;
      double (C3D10::*dNdq_[10][3])(double x, double y, double z) const;
      double (C3D10::*NA_[6])(double x, double y) const;
      double (C3D10::*dNAdq_[6][2])(double x, double y) const;
      int fI[4][6];

      double N1(double x, double y, double z) const {
	return (1-x-y-z)*(2*(1-x-y-z)-1);
      }

      double N2(double x, double y, double z) const {
	return x*(2*x-1);
      }

      double N3(double x, double y, double z) const {
return y*(2*y-1);
      }

      double N4(double x, double y, double z) const {
	return z*(2*z-1);
      }

      double N5(double x, double y, double z) const {
	return 4*(1-x-y-z)*x;
      }

      double N6(double x, double y, double z) const {
	return 4*x*y;
      }

      double N7(double x, double y, double z) const {
	return 4*(1-x-y-z)*y;
      }

      double N8(double x, double y, double z) const {
	return 4*(1-x-y-z)*z;
      }

      double N9(double x, double y, double z) const {
	return 4*x*z;
      }

      double N10(double x, double y, double z) const {
	return 4*y*z;
      }

      double dN1dx(double x, double y, double z) const {
	return 1-4*(1-x-y-z);
      }

      double dN1dy(double x, double y, double z) const {
	return 1-4*(1-x-y-z);
      }

      double dN1dz(double x, double y, double z) const {
	return 1-4*(1-x-y-z);
      }

      double dN2dx(double x, double y, double z) const {
	return 4*x-1;
      }

      double dN2dy(double x, double y, double z) const {
	return 0;
      }

      double dN2dz(double x, double y, double z) const {
	return 0;
      }

      double dN3dx(double x, double y, double z) const {
	return 0;
      }

      double dN3dy(double x, double y, double z) const {
	return 4*y-1;
      }

      double dN3dz(double x, double y, double z) const {
	return 0;
      }

      double dN4dx(double x, double y, double z) const {
	return 0;
      }

      double dN4dy(double x, double y, double z) const {
	return 0;
      }

      double dN4dz(double x, double y, double z) const {
	return 4*z-1;
      }

      double dN5dx(double x, double y, double z) const {
	return 4*((1-x-y-z)-x);
      }

      double dN5dy(double x, double y, double z) const {
	return -4*x;
      }

      double dN5dz(double x, double y, double z) const {
	return -4*x;
      }

      double dN6dx(double x, double y, double z) const {
	return 4*y;
      }

      double dN6dy(double x, double y, double z) const {
	return 4*x;
      }

      double dN6dz(double x, double y, double z) const {
	return 0;
      }

      double dN7dx(double x, double y, double z) const {
	return -4*y;
      }

      double dN7dy(double x, double y, double z) const {
	return 4*((1-x-y-z)-y);
      }

      double dN7dz(double x, double y, double z) const {
	return -4*y;
      }

      double dN8dx(double x, double y, double z) const {
	return -4*z;
      }

      double dN8dy(double x, double y, double z) const {
	return -4*z;
      }

      double dN8dz(double x, double y, double z) const {
	return 4*((1-x-y-z)-z);
      }

      double dN9dx(double x, double y, double z) const {
	return 4*z;
      }

      double dN9dy(double x, double y, double z) const {
	return 0;
      }

      double dN9dz(double x, double y, double z) const {
	return 4*x;
      }

      double dN10dx(double x, double y, double z) const {
	return 0;
      }

      double dN10dy(double x, double y, double z) const {
	return 4*z;
      }

      double dN10dz(double x, double y, double z) const {
	return 4*y;
      }

      double NA1(double x, double y) const {
        return 2*(0.5-x-y)*(1-x-y);
      }

      double NA2(double x, double y) const {
        return x*(2*x-1);
      }

      double NA3(double x, double y) const {
        return y*(2*y-1);
      }

      double NA4(double x, double y) const {
        return 4*x*(1-x-y);
      }

      double NA5(double x, double y) const {
        return 4*x*y;
      }

      double NA6(double x, double y) const {
        return 4*y*(1-x-y);
      }

      double dNA1dx(double x, double y) const {
        return 4*(x+y)-3;
      }

      double dNA1dy(double x, double y) const {
        return 4*(x+y)-3;
      }

      double dNA2dx(double x, double y) const {
        return 4*x-1;
      }

      double dNA2dy(double x, double y) const {
        return 0;
      }

      double dNA3dx(double x, double y) const {
        return 0;
      }

      double dNA3dy(double x, double y) const {
        return 4*y-1;
      }

      double dNA4dx(double x, double y) const {
        return 4*(1-x-y-x);
      }

      double dNA4dy(double x, double y) const {
        return -4*x;
      }

      double dNA5dx(double x, double y) const {
        return 4*y;
      }

      double dNA5dy(double x, double y) const {
        return 4*x;
      }

      double dNA6dx(double x, double y) const {
        return -4*y;
      }

      double dNA6dy(double x, double y) const {
        return 4*(1-x-y-y);
      }
  };

}

#endif

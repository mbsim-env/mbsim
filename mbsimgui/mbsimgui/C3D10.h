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

      int getNumberOfNodes() const override { return 10; }
      int getNumberOfIntegrationPoints() const override { return 4; }
      const fmatvec::Vec3& getNaturalCoordinates(int i) const override { return rN[i]; }
      const fmatvec::Vec3& getIntegrationPoint(int i) const override { return rI[i]; }
      double getWeight(int i) const override { return wI[i]; }
      int getNumberOfFaces() const { return 4; }
      int getNumberOfIndicesPerFace() const { return 3; }
      int getOmbvIndex(int i, int j) const { return indices[i][j]; }

    private:
      fmatvec::Vec3 rN[10], rI[4];
      double wI[4];
      int indices[4][3];
      double (C3D10::*N_[10])(double x, double y, double z) const;
      double (C3D10::*dNdq_[10][3])(double x, double y, double z) const;

      double NL1(double x, double y, double z) const {
	return 1-x-y-z;
      }
      double NL2(double x, double y, double z) const {
	return x;
      }
      double NL3(double x, double y, double z) const {
	return y;
      }
      double NL4(double x, double y, double z) const {
	return z;
      }
    
      double N1(double x, double y, double z) const {
	double NL = NL1(x,y,z);
	return NL*(2*NL-1);
      }

      double N2(double x, double y, double z) const {
	double NL = NL2(x,y,z);
	return NL*(2*NL-1);
      }

      double N3(double x, double y, double z) const {
	double NL = NL3(x,y,z);
	return NL*(2*NL-1);
      }

      double N4(double x, double y, double z) const {
	double NL = NL4(x,y,z);
	return NL*(2*NL-1);
      }

      double N5(double x, double y, double z) const {
	return 4*NL1(x,y,z)*NL2(x,y,z);
      }

      double N6(double x, double y, double z) const {
	return 4*NL2(x,y,z)*NL3(x,y,z);
      }

      double N7(double x, double y, double z) const {
	return 4*NL1(x,y,z)*NL3(x,y,z);
      }

      double N8(double x, double y, double z) const {
	return 4*NL1(x,y,z)*NL4(x,y,z);
      }

      double N9(double x, double y, double z) const {
	return 4*NL2(x,y,z)*NL4(x,y,z);
      }

      double N10(double x, double y, double z) const {
	return 4*NL3(x,y,z)*NL4(x,y,z);
      }

      double dN1dx(double x, double y, double z) const {
	return 1-4*NL1(x,y,z);
      }

      double dN1dy(double x, double y, double z) const {
	return 1-4*NL1(x,y,z);
      }

      double dN1dz(double x, double y, double z) const {
	return 1-4*NL1(x,y,z);
      }

      double dN2dx(double x, double y, double z) const {
	return 4*NL2(x,y,z)-1;
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
	return 4*NL3(x,y,z)-1;
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
	return 4*NL4(x,y,z)-1;
      }

      double dN5dx(double x, double y, double z) const {
	return 4*(NL1(x,y,z)-NL2(x,y,z));
      }

      double dN5dy(double x, double y, double z) const {
	return -4*NL2(x,y,z);
      }

      double dN5dz(double x, double y, double z) const {
	return -4*NL2(x,y,z);
      }

      double dN6dx(double x, double y, double z) const {
	return 4*NL3(x,y,z);
      }

      double dN6dy(double x, double y, double z) const {
	return 4*NL2(x,y,z);
      }

      double dN6dz(double x, double y, double z) const {
	return 0;
      }

      double dN7dx(double x, double y, double z) const {
	return -4*NL3(x,y,z);
      }

      double dN7dy(double x, double y, double z) const {
	return 4*(NL1(x,y,z)-NL3(x,y,z));
      }

      double dN7dz(double x, double y, double z) const {
	return -4*NL3(x,y,z);
      }

      double dN8dx(double x, double y, double z) const {
	return -4*NL4(x,y,z);
      }

      double dN8dy(double x, double y, double z) const {
	return -4*NL4(x,y,z);
      }

      double dN8dz(double x, double y, double z) const {
	return 4*(NL1(x,y,z)-NL4(x,y,z));
      }

      double dN9dx(double x, double y, double z) const {
	return 4*NL4(x,y,z);
      }

      double dN9dy(double x, double y, double z) const {
	return 0;
      }

      double dN9dz(double x, double y, double z) const {
	return 4*NL2(x,y,z);
      }

      double dN10dx(double x, double y, double z) const {
	return 0;
      }

      double dN10dy(double x, double y, double z) const {
	return 4*NL4(x,y,z);
      }

      double dN10dz(double x, double y, double z) const {
	return 4*NL3(x,y,z);
      }
  };

}

#endif

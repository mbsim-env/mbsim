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

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include <fmatvec.h>
#include <fstream>


namespace MBSim {

  class Translation {
    public:
      virtual ~Translation() {}
      virtual int getqSize() const = 0;
      virtual fmatvec::Vec operator()(const fmatvec::Vec &q, double t) = 0; 
  };

  class LinearTranslation : public Translation {
    private:
      fmatvec::Mat PJT;
    public:
      LinearTranslation() {} 
      LinearTranslation(const fmatvec::Mat &PJT_) { PJT = PJT_; } 

      const fmatvec::Mat& getPJT() const {return PJT;}
      void setPJT(const fmatvec::Mat &PJT_) {PJT = PJT_;}

      int getqSize() const {return PJT.cols();}

      fmatvec::Vec operator()(const fmatvec::Vec &q, double t) {
	return PJT*q(0,PJT.cols()-1);
      }; 
  };

  class Rotation {
    public:
      virtual ~Rotation() {}
      virtual int getqSize() const = 0;
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t) = 0; 
  };

  class Jacobian {
    public:
      virtual ~Jacobian() {}
      virtual int getuSize() const = 0;
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t) = 0; 
  };

  class ConstantJacobian : public Jacobian {
    private:
      fmatvec::Mat J;
    public:
      virtual int getuSize() const {return J.cols();}
      ConstantJacobian(const fmatvec::Mat &J_) {J = J_;}

    public:
       virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t) {return J;} 
  };

  class DerivativeOfJacobian {
    public:
      virtual ~DerivativeOfJacobian() {}
      virtual fmatvec::Mat operator()(const fmatvec::Vec &qd, const fmatvec::Vec &q, double t) = 0; 
  };

  class TimeDependentFunction {
    public:
      virtual ~TimeDependentFunction() {}
      virtual fmatvec::Vec operator()(double t) = 0; 
  };


  class RotationAboutFixedAxis: public Rotation {
    private:
      fmatvec::SqrMat APK;
      fmatvec::Vec a;
    public:
      RotationAboutFixedAxis() : APK(3), a(3) {}
      RotationAboutFixedAxis(const fmatvec::Vec &a_) : APK(3) { a = a_; } 

      const fmatvec::Vec& getAxisOfRotation() const {return a;}
      virtual int getqSize() const {return 1;}

      void setAxisOfRotation(const fmatvec::Vec& a_) {a = a_;}

      fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
  };

  class CardanAngles: public Rotation {
    private:
      fmatvec::SqrMat APK;
      fmatvec::Vec a;
    public:
      CardanAngles() : APK(3), a(3) {}
      CardanAngles(const fmatvec::Vec &a_) : APK(3) { a = a_; } 

      virtual int getqSize() const {return 3;}

      fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
  };

  class TCardanAngles : public Jacobian {
    private:
      int qSize, uSize;
      fmatvec::Mat T;

    public:
      TCardanAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      int getuSize() const {return uSize;}

      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
  };

  class TCardanAngles2 : public Jacobian {
    private:
      int qSize, uSize;
      fmatvec::Mat T;

    public:
      TCardanAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      int getuSize() const {return uSize;}

      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
  };

}

#endif

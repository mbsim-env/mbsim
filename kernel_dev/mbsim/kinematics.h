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

using namespace fmatvec;

namespace MBSim {

  class Translation {
    public:
      virtual ~Translation() {}
      virtual int getqSize() const = 0;
      virtual Vec operator()(const Vec &q, double t) = 0; 
      virtual void save(const std::string &path, std::ofstream& outputfile);
      virtual void load(const std::string &path, std::ifstream& outputfile);
  };

  class LinearTranslation : public Translation {
    private:
      Mat PJT;
    public:
      LinearTranslation() {} 
      LinearTranslation(const Mat &PJT_) { PJT = PJT_; } 

      const Mat& getPJT() const {return PJT;}
      void setPJT(const Mat &PJT_) {PJT = PJT_;}

      int getqSize() const {return PJT.cols();}

      Vec operator()(const Vec &q, double t) {
	return PJT*q(0,PJT.cols()-1);
      }; 

      void save(const std::string &path, std::ofstream& outputfile);
      void load(const std::string &path, std::ifstream& outputfile);
  };

  class Rotation {
    public:
      virtual ~Rotation() {}
      virtual int getqSize() const = 0;
      virtual SqrMat operator()(const Vec &q, double t) = 0; 
      virtual void save(const std::string &path, std::ofstream& outputfile);
      virtual void load(const std::string &path, std::ifstream& outputfile);
  };

  class Jacobian {
    public:
      virtual ~Jacobian() {}
      virtual int getuSize() const = 0;
      virtual Mat operator()(const Vec &q, double t) = 0; 
  };

  class ConstantJacobian : public Jacobian {
    private:
      Mat J;
    public:
      virtual int getuSize() const {return J.cols();}
      ConstantJacobian(const Mat &J_) {J = J_;}

    public:
       virtual Mat operator()(const Vec &q, double t) {return J;} 
  };

  class DerivativeOfJacobian {
    public:
      virtual ~DerivativeOfJacobian() {}
      virtual Mat operator()(const Vec &qd, const Vec &q, double t) = 0; 
  };

  class TimeDependentFunction {
    public:
      virtual ~TimeDependentFunction() {}
      virtual Vec operator()(double t) = 0; 
  };


  class RotationAboutFixedAxis: public Rotation {
    private:
      SqrMat APK;
      Vec a;
    public:
      RotationAboutFixedAxis() : APK(3), a(3) {}
      RotationAboutFixedAxis(const Vec &a_) : APK(3) { a = a_; } 

      const Vec& getAxisOfRotation() const {return a;}
      virtual int getqSize() const {return 1;}

      void setAxisOfRotation(const Vec& a_) {a = a_;}

      SqrMat operator()(const Vec &q, double t);

      void save(const std::string &path, std::ofstream& outputfile);
      void load(const std::string &path, std::ifstream& outputfile);
  };

  class CardanAngles: public Rotation {
    private:
      SqrMat APK;
      Vec a;
    public:
      CardanAngles() : APK(3), a(3) {}
      CardanAngles(const Vec &a_) : APK(3) { a = a_; } 

      virtual int getqSize() const {return 3;}

      SqrMat operator()(const Vec &q, double t);
  };

  class TCardanAngles : public Jacobian {
    private:
      int qSize, uSize;
      Mat T;

    public:
      TCardanAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,EYE) {}

      int getuSize() const {return uSize;}

      virtual Mat operator()(const Vec &q, double t);
  };

  class TCardanAngles2 : public Jacobian {
    private:
      int qSize, uSize;
      Mat T;

    public:
      TCardanAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,EYE) {}

      int getuSize() const {return uSize;}

      virtual Mat operator()(const Vec &q, double t);
  };

}

#endif

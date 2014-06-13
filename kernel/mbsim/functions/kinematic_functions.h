/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef _KINEMATIC_FUNCTIONS_H_
#define _KINEMATIC_FUNCTIONS_H_

#include "fmatvec/function.h"
#include "mbsim/objectfactory.h"
#include "mbsim/element.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Arg>
    class TranslationAlongXAxis : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r, a;
      public:
        TranslationAlongXAxis() { a.e(0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(0) = ToDouble<Arg>::cast(q);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return a; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongYAxis : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r, a;
      public:
        TranslationAlongYAxis() { a.e(1) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(1) = ToDouble<Arg>::cast(q);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return a; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongZAxis : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r, a;
      public:
        TranslationAlongZAxis() { a.e(2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(2) = ToDouble<Arg>::cast(q);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return a; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongAxesXY : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r;
        fmatvec::Mat3xV A;
      public:
        TranslationAlongAxesXY() : A(2) { A.e(0,0) = 1; A.e(1,1) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(0) = q.e(0);
          r.e(1) = q.e(1);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(2); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongAxesYZ : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r;
        fmatvec::Mat3xV A;
      public:
        TranslationAlongAxesYZ() : A(2) { A.e(1,0) = 1; A.e(2,1) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(1) = q.e(0);
          r.e(2) = q.e(1);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(2); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongAxesXZ : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r;
        fmatvec::Mat3xV A;
      public:
        TranslationAlongAxesXZ() : A(2) { A.e(0,0) = 1; A.e(2,1) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(0) = q.e(0);
          r.e(2) = q.e(1);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(2); }
        bool constParDer() const { return true; }
    };

  template<typename Arg>
    class TranslationAlongAxesXYZ : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 r;
        fmatvec::Mat3xV A;
      public:
        TranslationAlongAxesXYZ() : A(3) { A.e(0,0) = 1; A.e(1,1) = 1; A.e(2,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::Vec3 operator()(const Arg &q) { 
          r.e(0) = q.e(0);
          r.e(1) = q.e(1);
          r.e(2) = q.e(2);
          return r; 
        }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(3); }
        bool constParDer() const { return true; }
    };

  template<class Arg>
    class TranslationAlongFixedAxis : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        fmatvec::Vec3 a;
        fmatvec::Vec3 zeros(const typename fmatvec::Der<fmatvec::Vec3, Arg>::type &x) { return fmatvec::Vec3(x.rows()); }
      public:
        TranslationAlongFixedAxis() { }
        TranslationAlongFixedAxis(const fmatvec::Vec3 &a_) : a(a_) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::Vec3 operator()(const Arg &arg) { return a*arg; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return a; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(1); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, double>::type, double>::type parDerParDer(const double &arg) { throw MBSimError("parDerParDer is not available for given template parameters."); }
        bool constParDer() const { return true; }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"axisOfTranslation");
          a=FromMatStr<fmatvec::Vec3>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str());
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
    };

  template<>
  inline fmatvec::Vec3 TranslationAlongFixedAxis<double>::parDerParDer(const double &arg) { return fmatvec::Vec3(); }

  template<class Arg>
    class LinearTranslation : public fmatvec::Function<fmatvec::Vec3(Arg)> {
      private:
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type A;
        fmatvec::Vec3 b;
        fmatvec::Vec3 zeros(const typename fmatvec::Der<fmatvec::Vec3, Arg>::type &x) { return fmatvec::Vec3(x.rows()); }
      public:
        LinearTranslation() { }
        LinearTranslation(const typename fmatvec::Der<fmatvec::Vec3, Arg>::type &A_) : A(A_) { }
        LinearTranslation(const typename fmatvec::Der<fmatvec::Vec3, Arg>::type &A_, const fmatvec::Vec3 &b_) : A(A_), b(b_) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return A.cols(); }
        fmatvec::Vec3 operator()(const Arg &arg) { return A*arg+b; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDer(const Arg &arg) { return A; }
        typename fmatvec::Der<fmatvec::Vec3, Arg>::type parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename fmatvec::Der<fmatvec::Vec3, Arg>::type(A.rows(),A.cols()); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::Vec3, double>::type, double>::type parDerParDer(const double &arg) { throw MBSimError("parDerParDer is not available for given template parameters."); }
        bool constParDer() const { return true; }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"translationVectors");
          A=FromMatStr<typename fmatvec::Der<fmatvec::Vec3, Arg>::type>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str());
          e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"offset");
          b=e?FromMatStr<fmatvec::Vec3>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str()):zeros(A);
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
        void setSlope(const typename fmatvec::Der<fmatvec::Vec3, Arg>::type &A_) { A = A_; }
        void setIntercept(const fmatvec::Vec3 &b_) { b = b_; }
    };

  template<>
    inline fmatvec::Vec3 LinearTranslation<double>::parDerDirDer(const double &arg1Dir, const double &arg1) { return fmatvec::Vec3(); }
  template<>
    inline fmatvec::Vec3 LinearTranslation<double>::parDerParDer(const double &arg) { return fmatvec::Vec3(); }

  template<class Arg> 
    class RotationAboutXAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutXAxis() { a.e(0) = 1; A.e(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          A.e(1,1) = cosq;
          A.e(2,1) = sinq;
          A.e(1,2) = -sinq;
          A.e(2,2) = cosq;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) { return a; }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) { return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<class Arg> 
    class RotationAboutYAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutYAxis() { a.e(1) = 1; A.e(1,1) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          A.e(0,0) = cosq;
          A.e(2,0) = -sinq;
          A.e(0,2) = sinq;
          A.e(2,2) = cosq;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) { return a; }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) { return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<class Arg> 
    class RotationAboutZAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutZAxis() { a.e(2) = 1; A.e(2,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          A.e(0,0) = cosq;
          A.e(1,0) = sinq;
          A.e(0,1) = -sinq;
          A.e(1,1) = cosq;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) { return a; }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) { return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1); }
        bool constParDer() const { return true; }
    };

  template<class Arg> 
    class RotationAboutFixedAxis : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        RotationAboutFixedAxis() { }
        RotationAboutFixedAxis(const fmatvec::Vec3 &a_) : a(a_) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 1; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          const double onemcosq=1-cosq;
          const double a0a1=a.e(0)*a.e(1);
          const double a0a2=a.e(0)*a.e(2);
          const double a1a2=a.e(1)*a.e(2);
          A.e(0,0) = cosq+onemcosq*a.e(0)*a.e(0);
          A.e(1,0) = onemcosq*a0a1+a.e(2)*sinq;
          A.e(2,0) = onemcosq*a0a2-a.e(1)*sinq;
          A.e(0,1) = onemcosq*a0a1-a.e(2)*sinq;
          A.e(1,1) = cosq+onemcosq*a.e(1)*a.e(1);
          A.e(2,1) = onemcosq*a1a2+a.e(0)*sinq;
          A.e(0,2) = onemcosq*a0a2+a.e(1)*sinq;
          A.e(1,2) = onemcosq*a1a2-a.e(0)*sinq;
          A.e(2,2) = cosq+onemcosq*a.e(2)*a.e(2);
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) { return a; }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) { return typename fmatvec::Der<fmatvec::RotMat3, Arg>::type(1); }
        typename fmatvec::Der<typename fmatvec::Der<fmatvec::RotMat3, Arg>::type, Arg>::type parDerParDer(const Arg &arg) { throw MBSimError("parDerParDer is not available for given template parameters."); }
        bool constParDer() const { return true; }
        const fmatvec::Vec3& getAxisOfRotation() const { return a; }
        void setAxisOfRotation(const fmatvec::Vec3 &a_) { a = a_; }
        void initializeUsingXML(xercesc::DOMElement *element) {
          xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"axisOfRotation");
          a=FromMatStr<fmatvec::Vec3>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str());
        }
        xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent) { return 0; } 
    };

  template<>
  inline fmatvec::Vec3 RotationAboutFixedAxis<double>::parDerParDer(const double &arg) { return fmatvec::Vec3(); }

  template<class Arg> 
    class RotationAboutAxesXY : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXY() : J(2), Jd(2) { J.e(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);

          A.e(0,0) = cosb;
          A.e(1,0) = sina*sinb;
          A.e(2,0) = -cosa*sinb;
          A.e(1,1) = cosa;
          A.e(2,1) = sina;
          A.e(0,2) = sinb;
          A.e(1,2) = -sina*cosb;
          A.e(2,2) = cosa*cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q.e(0);
          J.e(1,1) = cos(a);
          J.e(2,1) = sin(a);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q.e(0);
          double ad = qd.e(0);
          Jd.e(1,1) = -sin(a)*ad;
          Jd.e(2,1) = cos(a)*ad;
          return Jd;
        }
    };

  template<class Arg> 
    class RotationAboutAxesYZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesYZ() : J(2), Jd(2) { J.e(1,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double b=q.e(0);
          double g=q.e(1);
          double cosb = cos(b);
          double sinb = sin(b);
          double cosg = cos(g);
          double sing = sin(g);

          A.e(0,0) = cosb*cosg;
          A.e(1,0) = sing;
          A.e(2,0) = -sinb*cosg;
          A.e(0,1) = -cosb*sing;
          A.e(1,1) = cosg;
          A.e(2,1) = sinb*sing;
          A.e(0,2) = sinb;
          A.e(2,2) = cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double beta = q.e(0);
          J.e(0,1) = sin(beta);
          J.e(2,1) = cos(beta);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double beta = q.e(0);
          double betad = qd.e(0);
          Jd.e(0,1) = cos(beta)*betad;
          Jd.e(2,1) = -sin(beta)*betad;
          return Jd;
        }
    };

  template<class Arg> 
    class RotationAboutAxesXZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXZ() : J(2), Jd(2) { J.e(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 2; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);

          A.e(0,0) = cosb;
          A.e(1,0) = cosa*sinb;
          A.e(2,0) = sina*sinb;
          A.e(0,1) = -sinb;
          A.e(1,1) = cosa*cosb;
          A.e(2,1) = sina*cosb;
          A.e(1,2) = -sina;
          A.e(2,2) = cosa;

          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q.e(0);
          J.e(1,1) = -sin(a);
          J.e(2,1) = cos(a);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q.e(0);
          double ad = qd.e(0);
          Jd.e(1,1) = -cos(a)*ad;
          Jd.e(2,1) = -sin(a)*ad;
          return Jd;
        }
    };

  /*!
   * \brief rotation class for rotation about all three axis using the cardan description
   */
  template<class Arg> 
    class RotationAboutAxesXYZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXYZ() : J(3), Jd(3) { J.e(0,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double g=q.e(2);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          double cosg = cos(g);
          double sing = sin(g);
          A.e(0,0) = cosb*cosg;
          A.e(1,0) = sina*sinb*cosg+cosa*sing;
          A.e(2,0) = -cosa*sinb*cosg+sina*sing;
          A.e(0,1) = -cosb*sing;
          A.e(1,1) = -sing*sinb*sina+cosa*cosg;
          A.e(2,1) = cosa*sinb*sing+sina*cosg;
          A.e(0,2) = sinb;
          A.e(1,2) = -sina*cosb;
          A.e(2,2) = cosa*cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          //J.e(0,0) = 1;
          //J.e(0,1) = 0;
          J.e(0,2) = sin(b);
          //J.e(1,0) = 0;
          J.e(1,1) = cosa;
          J.e(1,2) = -sina*cosb;
          //J.e(2,0) = 0;
          J.e(2,1) = sina;
          J.e(2,2) = cosa*cosb;
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double ad = qd.e(0);
          double bd = qd.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          //Jd.e(0,0) = 0;
          //Jd.e(0,1) = 0;
          Jd.e(0,2) = cosb*bd;
          //Jd.e(1,0) = 0;
          Jd.e(1,1) = -sina*ad;
          Jd.e(1,2) = -cosa*cosb*ad + sina*sinb*bd;
          //Jd.e(2,0) = 0;
          Jd.e(2,1) = cosa*ad;
          Jd.e(2,2) = -sina*cosb*ad - cosa*sinb*bd;
          return Jd;
        }
    };

    template<class Arg> 
    class RotationAboutAxesXYZ2 : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesXYZ2() : J(3), Jd(3) { J.e(2,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double g=q.e(2);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          double cosg = cos(g);
          double sing = sin(g);
          A.e(0,0) = cosb*cosg;
          A.e(1,0) = sina*sinb*cosg+cosa*sing;
          A.e(2,0) = -cosa*sinb*cosg+sina*sing;
          A.e(0,1) = -cosb*sing;
          A.e(1,1) = -sing*sinb*sina+cosa*cosg;
          A.e(2,1) = cosa*sinb*sing+sina*cosg;
          A.e(0,2) = sinb;
          A.e(1,2) = -sina*cosb;
          A.e(2,2) = cosa*cosb;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double b = q.e(1);
          double g = q.e(2);
          J.e(0,0) = cos(b)*cos(g);
          J.e(0,1) = sin(g);
          //J.e(0,2) = 0;
          J.e(1,0) = -cos(b)*sin(g);
          J.e(1,1) = cos(g);
          //J.e(1,2) = 0;
          J.e(2,0) = sin(b);
          //J.e(2,1) = 0;
          //J.e(2,2) = 1;
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double b = q.e(1);
          double g = q.e(2);
          double bd = qd.e(1);
          double gd = qd.e(2);
          Jd.e(0,0) = -sin(b)*cos(g)*bd - cos(b)*sin(g)*gd;
          Jd.e(0,1) = cos(g)*gd;
          //Jd.e(0,2) = 0;
          Jd.e(1,0) = sin(b)*sin(g)*bd - cos(b)*cos(g)*gd;
          Jd.e(1,1) = -sin(g)*gd;
          //Jd.e(1,2) = 0; 
          Jd.e(2,0) = cos(b)*bd;
          //Jd.e(2,1) = 0;
          //Jd.e(2,2) = 0;
          return Jd;
        }
    };
  
  template<class Arg> 
    class RotationAboutAxesZXZ : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesZXZ() : J(3), Jd(3) { J.e(2,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double psi=q.e(0);
          double theta=q.e(1);
          double phi=q.e(2);
          double spsi = sin(psi);
          double stheta = sin(theta);
          double sphi = sin(phi);
          double cpsi = cos(psi);
          double ctheta = cos(theta);
          double cphi = cos(phi);
          A.e(0,0) = cpsi*cphi-spsi*ctheta*sphi;
          A.e(1,0) = spsi*cphi+cpsi*ctheta*sphi;
          A.e(2,0) = stheta*sphi;
          A.e(0,1) = -cpsi*sphi-spsi*ctheta*cphi;
          A.e(1,1) = -spsi*sphi+cpsi*ctheta*cphi;
          A.e(2,1) = stheta*cphi;
          A.e(0,2) = spsi*stheta;
          A.e(1,2) = -cpsi*stheta;
          A.e(2,2) = ctheta;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double psi=q.e(0);
          double theta=q.e(1);
          //J.e(0,0) = 0;
          J.e(0,1) = cos(psi);
          J.e(0,2) = sin(psi)*sin(theta);
          //J.e(1,0) = 0;
          J.e(1,1) = sin(psi);
          J.e(1,2) = -cos(psi)*sin(theta);
          //J.e(2,0) = 1;
          //J.e(2,1) = 1;
          J.e(2,2) = cos(theta);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double psi=q.e(0);
          double theta=q.e(1);
          double psid=qd.e(0);
          double thetad=qd.e(1);
          Jd.e(0,1) = -sin(psi)*psid;
          Jd.e(0,2) = cos(psi)*sin(theta)*psid + sin(psi)*cos(theta)*thetad;
          Jd.e(1,1) = cos(psi)*psid;
          Jd.e(1,2) = sin(psi)*sin(theta)*psid - cos(psi)*cos(theta)*thetad;
          Jd.e(2,2) = -sin(theta)*thetad;
          return Jd;
        }
    };

  template<class Arg> 
    class RotationAboutAxesZYX : public fmatvec::Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        RotationAboutAxesZYX() : J(3), Jd(3) { J.e(2,0) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double a=q.e(0);
          double b=q.e(1);
          double g=q.e(2);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          double cosg = cos(g);
          double sing = sin(g);
          A.e(0,0) = cosa*cosb;
          A.e(1,0) = sina*cosb;
          A.e(2,0) = -sinb;
          A.e(0,1) = -sina*cosg+cosa*sinb*sing;
          A.e(1,1) = cosa*cosg+sina*sinb*sing;
          A.e(2,1) = cosb*sing;
          A.e(0,2) = sina*sing+cosa*sinb*cosg;
          A.e(1,2) = -cosa*sing+sina*sinb*cosg;
          A.e(2,2) = cosb*cosg;
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          //J.e(0,0) = 0;
          J.e(0,1) = -sina;
          J.e(0,2) = cosa*cosb;
          //J.e(1,0) = 0;
          J.e(1,1) = cosa;
          J.e(1,2) = sina*cosb;
          //J.e(2,0) = 1;
          //J.e(2,1) = 0;
          J.e(2,2) = -sin(b);
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          double a = q.e(0);
          double b = q.e(1);
          double ad = qd.e(0);
          double bd = qd.e(1);
          double cosa = cos(a);
          double sina = sin(a);
          double cosb = cos(b);
          double sinb = sin(b);
          Jd.e(0,1) = -cosa*ad;
          Jd.e(0,2) = -sina*cosb*ad - cosa*sinb*bd;
          Jd.e(1,1) = -sina*ad;
          Jd.e(1,2) = cosa*cosb*ad - sina*sinb*bd;
          Jd.e(2,2) = -cosb*bd;
          return Jd;
        }
    };

  template<class Arg> 
    class RotationAboutAxesXYZMapping : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        RotationAboutAxesXYZMapping() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::MatV operator()(const Arg &q) {
          double alpha = q.e(0);
          double beta = q.e(1);
          double cos_beta = cos(beta);
          double sin_beta = sin(beta);
          double cos_alpha = cos(alpha);
          double sin_alpha = sin(alpha);
          double tan_beta = sin_beta/cos_beta;
          T.e(0,1) = tan_beta*sin_alpha;
          T.e(0,2) = -tan_beta*cos_alpha;
          T.e(1,1) = cos_alpha;
          T.e(1,2) = sin_alpha;
          T.e(2,1) = -sin_alpha/cos_beta;
          T.e(2,2) = cos_alpha/cos_beta;
          return T;
        }
    };

  template<class Arg> 
    class RotationAboutAxesXYZMapping2 : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        RotationAboutAxesXYZMapping2() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::MatV operator()(const Arg &q) {
          double beta = q.e(1);
          double gamma = q.e(2);
          double cos_beta = cos(beta);
          double sin_beta = sin(beta);
          double cos_gamma = cos(gamma);
          double sin_gamma = sin(gamma);
          double tan_beta = sin_beta/cos_beta;
          T.e(0,0) = cos_gamma/cos_beta;
          T.e(0,1) = -sin_gamma/cos_beta;
          T.e(1,0) = sin_gamma;
          T.e(1,1) = cos_gamma;
          T.e(2,0) = -cos_gamma*tan_beta;
          T.e(2,1) = sin_gamma*tan_beta;
          return T;
        }
    };

  template<class Arg> 
    class RotationAboutAxesZXZMapping : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        RotationAboutAxesZXZMapping() : T(3,3) { T.e(0,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::MatV operator()(const Arg &q) {
          double psi = q.e(0);
          double theta = q.e(1);
          double cos_theta = cos(theta);
          double sin_theta = sin(theta);
          double cos_psi = cos(psi);
          double sin_psi = sin(psi);
          double tan_theta = sin_theta/cos_theta;

          T.e(0,0) = -sin_psi/tan_theta;
          T.e(0,1) = cos_psi/tan_theta;
          T.e(1,0) = cos_psi;
          T.e(1,1) = sin_psi;
          T.e(2,0) = sin_psi/sin_theta;
          T.e(2,1) = -cos_psi/sin_theta;

          return T;
        }
    };

  template<class Arg> 
    class RotationAboutAxesZXZMapping2 : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        RotationAboutAxesZXZMapping2() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::MatV operator()(const Arg &q) {
          double theta = q.e(1);
          double phi = q.e(2);
          double cos_theta = cos(theta);
          double sin_theta = sin(theta);
          double cos_phi = cos(phi);
          double sin_phi = sin(phi);
          double tan_theta = sin_theta/cos_theta;

          T.e(0,0) = sin_phi/sin_theta;
          T.e(0,1) = cos_phi/sin_theta;
          T.e(1,0) = cos_phi;
          T.e(1,1) = -sin_phi;
          T.e(2,0) = -sin_phi/tan_theta;
          T.e(2,1) = -cos_phi/tan_theta;

          return T;
        }
    };

  template<class Arg> 
    class RotationAboutAxesZYXMapping : public fmatvec::Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        RotationAboutAxesZYXMapping() : T(3,3) { T.e(0,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const { return 3; }
        fmatvec::MatV operator()(const Arg &q) {
          double alpha = q.e(0);
          double beta = q.e(1);
          double cos_beta = cos(beta);
          double sin_beta = sin(beta);
          double cos_alpha = cos(alpha);
          double sin_alpha = sin(alpha);
          double tan_beta = sin_beta/cos_beta;
          T.e(0,0) = cos_alpha*tan_beta;
          T.e(0,1) = sin_alpha*tan_beta;
          T.e(1,0) = -sin_alpha;
          T.e(1,1) = cos_alpha;
          T.e(2,0) = cos_alpha/cos_beta;
          T.e(2,1) = sin_alpha/cos_beta;
          return T;
        }
    };

}

#endif

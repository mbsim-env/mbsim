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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "mbsim/utils/function.h"

namespace MBSim {

  template <class Arg>
    class ToDouble {
    };

  template <>
    class ToDouble<double> {
      public:
        static double cast(const double &x) {
          return x;
        }
    };

  template <class Col>
    class ToDouble<fmatvec::Vector<Col,double> > {
      public:
        static double cast(const fmatvec::Vector<Col,double> &x) {
          return x.e(0); 
        }
    };

  template<class Arg> 
    class FRotationAboutFixedAxis : public Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Vec3 a;
      public:
        FRotationAboutFixedAxis(const fmatvec::Vec3 &a_=fmatvec::Vec3()) : a(a_) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 1;
        }
        fmatvec::RotMat3 operator()(const Arg &q) {
          double alpha = ToDouble<Arg>::cast(q);
          const double cosq=cos(alpha);
          const double sinq=sin(alpha);
          const double onemcosq=1-cosq;
          const double a0a1=a(0)*a(1);
          const double a0a2=a(0)*a(2);
          const double a1a2=a(1)*a(2);
          A.e(0,0) = cosq+onemcosq*a(0)*a(0);
          A.e(1,0) = onemcosq*a0a1+a(2)*sinq;
          A.e(2,0) = onemcosq*a0a2-a(1)*sinq;
          A.e(0,1) = onemcosq*a0a1-a(2)*sinq;
          A.e(1,1) = cosq+onemcosq*a(1)*a(1);
          A.e(2,1) = onemcosq*a1a2+a(0)*sinq;
          A.e(0,2) = onemcosq*a0a2+a(1)*sinq;
          A.e(1,2) = onemcosq*a1a2-a(0)*sinq;
          A.e(2,2) = cosq+onemcosq*a(2)*a(2);
          return A;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDer(const Arg &q) {
          return a;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          return fmatvec::Vec3();
        }
        const fmatvec::Vec3& getAxisOfRotation() const { return a; }
        void setAxisOfRotation(const fmatvec::Vec3 &a_) { a = a_; }
    };

  template<class Arg> 
    class FRotationAboutAxesXYZ : public Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        FRotationAboutAxesXYZ() : J(3), Jd(3) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
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
          J.e(0,0) = 1;
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
    class FRotationAboutAxesZXZ : public Function<fmatvec::RotMat3(Arg)> {
      private:
        fmatvec::RotMat3 A;
        fmatvec::Mat3xV J, Jd;
      public:
        FRotationAboutAxesZXZ() : J(3), Jd(3) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
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
          throw std::runtime_error("FRotationAboutAxesZXZ::parDer() not yet implemented.");
          return J;
        }
        typename fmatvec::Der<fmatvec::RotMat3, Arg>::type parDerDirDer(const Arg &qd, const Arg &q) {
          throw std::runtime_error("FRotationAboutAxesZXZ::parDerDirDer() not yet implemented.");
          return Jd;
        }
    };

  template<class Arg> 
    class TCardanAngles : public Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TCardanAngles() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
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
    class TCardanAngles2 : public Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TCardanAngles2() : T(3,3,fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
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
    class TEulerAngles : public Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TEulerAngles() : T(3,3) { T(0,2) = 1; }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          throw std::runtime_error("TEulerAngles::operator() not yet implemented.");
          return T;
        }
    };

  template<class Arg> 
    class TEulerAngles2 : public Function<fmatvec::MatV(Arg)> {
      private:
        fmatvec::MatV T;
      public:
        TEulerAngles2() : T(fmatvec::Eye()) { }
        typename fmatvec::Size<Arg>::type getArgSize() const {
          return 3;
        }
        fmatvec::MatV operator()(const Arg &q) {
          throw std::runtime_error("TEulerAngles2::operator() not yet implemented.");
          return T;
        }
    };

  class Translation {
    protected:
      fmatvec::Vec3 r, j, jd;
      fmatvec::Mat3xV J, Jd;
      fmatvec::MatV T;

    public:
      virtual ~Translation() { }

      virtual void init();

      virtual int getqSize() const = 0;
      virtual int getuSize() const { return getqSize(); }

      virtual bool isIndependent() const { return false; }

      const fmatvec::Vec3 getPosition() const {return r;}
      const fmatvec::Mat3xV getJacobian() const {return J;}
      const fmatvec::Vec3 getGuidingVelocity() const {return j;}
      const fmatvec::Mat3xV getDerivativeOfJacobian() const {return Jd;}
      const fmatvec::Vec3 getDerivativeOfGuidingVelocity() const {return jd;}
      const fmatvec::MatV getT() const {return T;}

      void updateStateDependentVariables(const fmatvec::VecV &q, const double &t);
      void updateStateDerivativeDependentVariables(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t);

      virtual void updatePosition(const fmatvec::VecV &q, const double &t) { }
      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { }
      virtual void updateT(const fmatvec::VecV &q, const double &t) { }
      virtual void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { }
      virtual void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }
  };

//  class GeneralTranslation : public Translation {
//    protected:
//      Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr;
//      Function<fmatvec::MatV(fmatvec::VecV, double)> *fT;
//      fmatvec::Mat3xV drdq, dotdrdq;
//      fmatvec::MatV dotT;
//
//    public:
//      GeneralTranslation(Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr_=0, Function<fmatvec::MatV(fmatvec::VecV, double)> *fT_=0) : fr(fr_), fT(fT_) { }
//
//      virtual ~GeneralTranslation() { delete fr; delete fT; }
//
//      virtual void init();
//
//      virtual int getqSize() const { return fr->getArg1Size(); }
//      virtual int getuSize() const { return (*fT)(fmatvec::Vec3(),0).cols(); }
//
//      virtual void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(q,t); }
//      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { drdq = fr->parDer1(q,t); J = drdq*T; }
//      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fr->parDer2(q,t); }
//      virtual void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q,t); }
//      virtual void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { dotdrdq = fr->parDer1DirDer1(qd,q,t)+fr->parDer1ParDer2(q,t); dotT = fT->dirDer1(qd,q,t) + fT->parDer2(q,t); Jd = dotdrdq*T + drdq*dotT; }
//      virtual void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = fr->parDer2DirDer1(qd,q,t) + fr->parDer2ParDer2(q,t); }
//  };

  class GeneralTranslation : public Translation {
    protected:
      Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr;
      fmatvec::Mat3xV drdq, dotdrdq;

    public:
      GeneralTranslation(Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr_=0) : fr(fr_) { }

      ~GeneralTranslation() { delete fr; }

      int getqSize() const { return fr->getArg1Size(); }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(q,t); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { drdq = fr->parDer1(q,t); J = drdq; }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fr->parDer2(q,t); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { dotdrdq = fr->parDer1DirDer1(qd,q,t)+fr->parDer1ParDer2(q,t); Jd = dotdrdq; }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = fr->parDer2DirDer1(qd,q,t) + fr->parDer2ParDer2(q,t); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
 };

  class StateDependentTranslation : public Translation {
    protected:
      Function<fmatvec::Vec3(fmatvec::VecV)> *fr;

    public:
      StateDependentTranslation(Function<fmatvec::Vec3(fmatvec::VecV)> *fr_=0) : fr(fr_) { }

      ~StateDependentTranslation() { delete fr; }

      int getqSize() const { return fr->getArgSize(); }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(q); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = fr->parDer(q); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { Jd = fr->parDerDirDer(qd,q); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TimeDependentTranslation : public Translation {
    protected:
      Function<fmatvec::Vec3(double)> *fr;

    public:
      TimeDependentTranslation(Function<fmatvec::Vec3(double)> *fr_=0) : fr(fr_) { }

      ~TimeDependentTranslation() { delete fr; }

      bool isIndependent() const { return true; }

      int getqSize() const { return 0; }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fr->parDer(t); }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = fr->parDerParDer(t); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(1) = q(0); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInZDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(2) = q(0); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(1) = q(1); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXZDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(2) = q(1); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYZDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(1) = q(0); r(2) = q(1); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYZDirection : public Translation {
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 3;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(1) = q(1); r(2) = q(2); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class LinearTranslation : public Translation {
    private:
      fmatvec::Mat3xV D;

    public:

      LinearTranslation() { }
      LinearTranslation(const fmatvec::Mat3xV &D_) : D(D_) { }

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return D.cols();}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = D*q; }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class StateDependentLinearTranslation : public Translation {
    protected:
      Function<fmatvec::VecV(fmatvec::VecV)> *fq;
      fmatvec::Mat3xV D;

    public:
      StateDependentLinearTranslation() : fq(0) { }
      StateDependentLinearTranslation(Function<fmatvec::VecV(fmatvec::VecV)> *fq_, const fmatvec::Mat3xV &D_) : fq(fq_), D(D_) { }
      ~StateDependentLinearTranslation() { delete fq; }

      void init();

      int getqSize() const { return D.cols(); }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = D*(*fq)(q); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = D*fq->parDer(q); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { Jd = D*fq->parDerDirDer(qd,q); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TimeDependentLinearTranslation : public Translation {
    protected:
      Function<fmatvec::VecV(double)> *fq;
      fmatvec::Mat3xV D;

    public:
      TimeDependentLinearTranslation() : fq(0) { }
      TimeDependentLinearTranslation(Function<fmatvec::VecV(double)> *fq_, const fmatvec::Mat3xV &D_) : fq(fq_), D(D_) { }
      ~TimeDependentLinearTranslation() { delete fq; }

      void init();

      int getqSize() const { return 0; }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = D*(*fq)(t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = D*fq->parDer(t); }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = D*fq->parDerParDer(t); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class Rotation {
    protected:
      fmatvec::RotMat3 A;
      fmatvec::Vec3 j, jd;
      fmatvec::Mat3xV J, Jd;
      fmatvec::MatV T;
      bool KOSY;

    public:
      Rotation() : KOSY(false) { }
      virtual ~Rotation() { }

      virtual void init();

      virtual int getqSize() const = 0;
      virtual int getuSize() const { return getqSize(); }

      virtual bool isIndependent() const { return false; }

      const fmatvec::RotMat3 getOrientation() const {return A;}
      const fmatvec::Mat3xV getJacobian() const {return J;}
      const fmatvec::Vec3 getGuidingVelocity() const {return j;}
      const fmatvec::Mat3xV getDerivativeOfJacobian() const {return Jd;}
      const fmatvec::Vec3 getDerivativeOfGuidingVelocity() const {return jd;}
      const fmatvec::MatV getT() const {return T;}

      void updateStateDependentVariables(const fmatvec::VecV &q, const double &t);
      void updateStateDerivativeDependentVariables(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t);

      virtual void updateOrientation(const fmatvec::VecV &q, const double &t) { } 
      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { }
      virtual void updateT(const fmatvec::VecV &q, const double &t) { }
      virtual void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { }
      virtual void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { }

      void setKOSY(bool KOSY_) { KOSY = KOSY_; }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }
 };

//  class Rotation {
//    protected:
//      Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA;
//      Function<fmatvec::MatV(fmatvec::VecV, double)> *fT;
//      fmatvec::RotMat3 A;
//      fmatvec::Vec3 j, jd;
//      fmatvec::Mat3xV J, Jd, dAdq, dotdAdq;
//      fmatvec::MatV T, dotT;
//
//    public:
//      Rotation(Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA_=0, Function<fmatvec::MatV(fmatvec::VecV, double)> *fT_=0) : fA(fA_), fT(fT_) { }
//
//      virtual ~Rotation() {delete fA; delete fT;}
//
//      virtual void init();
//
//      virtual int getqSize() const { return fA->getArg1Size(); }
//      virtual int getuSize() const { return (*fT)(fmatvec::Vec3(),0).cols(); }
//
//      virtual bool isIndependent() const { return false; }
//
//      const fmatvec::RotMat3 getOrientation() const {return A;}
//      const fmatvec::Mat3xV getJacobian() const {return J;}
//      const fmatvec::Vec3 getGuidingVelocity() const {return j;}
//      const fmatvec::Mat3xV getDerivativeOfJacobian() const {return Jd;}
//      const fmatvec::Vec3 getDerivativeOfGuidingVelocity() const {return jd;}
//      const fmatvec::MatV getT() const {return T;}
//
//      void updateStateDependentVariables(const fmatvec::VecV &q, const double &t);
//      void updateStateDerivativeDependentVariables(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t);
//
//      virtual void updateOrientation(const fmatvec::VecV &q, const double &t) { A = (*fA)(q,t); }
//      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { dAdq = fA->parDer1(q,t); J = dAdq*T; }
//      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fA->parDer2(q,t); }
//      virtual void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q,t); }
//      virtual void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { dotdAdq = fA->parDer1DirDer1(qd,q,t)+fA->parDer1ParDer2(q,t); dotT = fT->dirDer1(qd,q,t) + fT->parDer2(q,t); Jd = dotdAdq*T + dAdq*dotT; }
//      virtual void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = fA->parDer2DirDer1(qd,q,t) + fA->parDer2ParDer2(q,t); }
//  };

  class GeneralRotation : public Rotation {
    protected:
      Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA;
      fmatvec::Mat3xV dAdq, dotdAdq;

    public:
      GeneralRotation(Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA_=0) : fA(fA_) { }

      ~GeneralRotation() { delete fA; }

      int getqSize() const { return fA->getArg1Size(); }

      bool isIndependent() const { return false; }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = (*fA)(q,t); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { dAdq = fA->parDer1(q,t); J = dAdq; }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fA->parDer2(q,t); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { dotdAdq = fA->parDer1DirDer1(qd,q,t)+fA->parDer1ParDer2(q,t); Jd = dotdAdq; }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { jd = fA->parDer2DirDer1(qd,q,t) + fA->parDer2ParDer2(q,t); }
  };

  class RotationAboutXAxis : public Rotation {
    public:
      bool isIndependent() const { return true; }

      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutYAxis : public Rotation {
    public:
      bool isIndependent() const { return true; }

      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutZAxis : public Rotation {
    public:
      bool isIndependent() const { return true; }

      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXY: public Rotation {
    public:
      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXZ: public Rotation {
    public:
      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesYZ: public Rotation {
    public:
      bool isIndependent() const { return true; }

      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXYZ: public Rotation {
    private:
      FRotationAboutAxesXYZ<fmatvec::VecV> f;
    public:
      bool isIndependent() const { return true; }

      int getqSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f(q); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = f.parDer(q); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { Jd = f.parDerDirDer(qd,q); }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutFixedAxis : public Rotation {
    private:
      FRotationAboutFixedAxis<fmatvec::VecV> f;

    public:

      RotationAboutFixedAxis(const fmatvec::Vec3 &a=fmatvec::Vec3()) : f(a) { }

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f(q); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);

      const fmatvec::Vec3& getAxisOfRotation() const { return f.getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec3 &a) { f.setAxisOfRotation(a); }
  };

  class StateDependentRotationAboutFixedAxis : public Rotation {
    private:
      Function<double(fmatvec::VecV)> *falpha;
      FRotationAboutFixedAxis<double> f;

    public:
      StateDependentRotationAboutFixedAxis(Function<double(fmatvec::VecV)> *falpha_=0, const fmatvec::Vec3 &a=fmatvec::Vec3()) : falpha(falpha_), f(a) { }

      ~StateDependentRotationAboutFixedAxis() { delete falpha; }

      void init();

      int getqSize() const { return 1; }

      bool isIndependent() const { return true; }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f((*falpha)(q)); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = getAxisOfRotation()*falpha->parDer(q); }
      void updateDerivativeOfJacobian(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) { Jd = getAxisOfRotation()*falpha->parDerDirDer(qd,q); }

      const fmatvec::Vec3& getAxisOfRotation() const { return f.getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec3 &a) { f.setAxisOfRotation(a); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TimeDependentRotationAboutFixedAxis : public Rotation {
    protected:
      Function<double(double)> *falpha;
      FRotationAboutFixedAxis<double> f;

    public:
      TimeDependentRotationAboutFixedAxis(Function<double(double)> *falpha_=0, const fmatvec::Vec3 &a=fmatvec::Vec3()) : falpha(falpha_), f(a) { }

      ~TimeDependentRotationAboutFixedAxis() { delete falpha; }

      int getqSize() const { return 0; }

      bool isIndependent() const { return true; }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f((*falpha)(t)); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = getAxisOfRotation()*falpha->parDer(t); }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) {jd = getAxisOfRotation()*falpha->parDerParDer(t); }

      const fmatvec::Vec3& getAxisOfRotation() const { return f.getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec3 &a) { f.setAxisOfRotation(a); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class CardanAngles : public Rotation {
    private:
      FRotationAboutAxesXYZ<fmatvec::VecV> fA;
      Function<fmatvec::MatV(fmatvec::VecV)> *fT;
    public:
      ~CardanAngles() { delete fT; }

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 3;}
      int getuSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA(q); }
      void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q); }

      void setKOSY(bool KOSY_) { KOSY = KOSY_; }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TimeDependentCardanAngles : public Rotation {
    protected:
      Function<fmatvec::VecV(double)> *fangles;
      FRotationAboutAxesXYZ<fmatvec::VecV> fA;
      TCardanAngles<fmatvec::VecV> fT;

    public:
      TimeDependentCardanAngles(Function<fmatvec::VecV(double)> *fangles_=0) : fangles(fangles_) { }

      ~TimeDependentCardanAngles() { delete fangles; }

      int getqSize() const { return 0; }

      bool isIndependent() const { return true; }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA((*fangles)(t)); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { j = fangles->parDer(t); }
      void updateDerivativeOfGuidingVelocity(const fmatvec::VecV &qd, const fmatvec::VecV &q, const double &t) {jd = fangles->parDerParDer(t); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class EulerAngles : public Rotation {
    private:
      FRotationAboutAxesZXZ<fmatvec::VecV> fA;
      TEulerAngles<fmatvec::VecV> fT;
    public:

      void init();

      bool isIndependent() const { return true; }

      int getqSize() const {return 3;}
      int getuSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA(q); }
      void updateT(const fmatvec::VecV &q, const double &t) { T = fT(q); }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

}

#endif /* _KINEMATICS_H_ */


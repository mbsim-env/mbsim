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

#include "mbsim/utils/function_library.h"

namespace MBSim {

  class Translation {
    protected:
      fmatvec::Vec3 r, v, jh, jb;
      fmatvec::Mat3xV J;
      fmatvec::MatV T;
      fmatvec::VecV qd;

    public:
      virtual ~Translation() { }

      virtual void init();

      virtual int getqSize() const = 0;
      virtual int getuSize() const { return getqSize(); }

      const fmatvec::Vec3 getPosition() const {return r;}
      const fmatvec::Vec3 getVelocity() const {return v;}
      const fmatvec::Mat3xV getJacobian() const {return J;}
      const fmatvec::Vec3 getGuidingVelocity() const {return jh;}
      const fmatvec::Vec3 getGyroscopicAcceleration() const {return jb;}
      const fmatvec::MatV getT() const {return T;}
      const fmatvec::VecV getqd() const {return qd;}

      virtual void updatePosition(const fmatvec::VecV &q, const double &t) { }
      virtual void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = J*u + jh; }
      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { }
      virtual void updateT(const fmatvec::VecV &q, const double &t) { }
      virtual void updateqd(const fmatvec::VecV &u) { qd = T*u; }
      virtual void updateStateDependentVariables(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t);

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }
  };

//  class GeneralTranslation : public Translation {
//    protected:
//      Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr;
//      Function<fmatvec::MatV(fmatvec::VecV, double)> *fT;
//      fmatvec::Mat3xV drdq;
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
//      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(q,t); }
//      void updateJacobian(const fmatvec::VecV &q, const double &t) { drdq = fr->parDer1(q,t); J = drdq*T; }
//      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { j = ((fr->parDer1DirDer1(qd,q,t)+fr->parDer1ParDer2(q,t))*T + drdq*(fT->dirDer1(qd,q,t) + fT->parDer2(q,t)))*u + fr->parDer2DirDer1(qd,q,t) + fr->parDer2ParDer2(q,t); }
//      void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q,t); }
//  };

  class GeneralTranslation : public Translation {
    protected:
      Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr;

    public:
      GeneralTranslation(Function<fmatvec::Vec3(fmatvec::VecV, double)> *fr_=0) : fr(fr_) { }

      ~GeneralTranslation() { delete fr; }

      int getqSize() const { return fr->getArg1Size(); }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(q,t); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = fr->parDer1(q,t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = fr->parDer2(q,t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = (fr->parDer1DirDer1(qd,q,t)+fr->parDer1ParDer2(q,t))*u + fr->parDer2DirDer1(qd,q,t) + fr->parDer2ParDer2(q,t); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

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
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = fr->parDer(q); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = fr->parDerDirDer(qd,q)*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TimeDependentTranslation : public Translation {
    protected:
      Function<fmatvec::Vec3(double)> *fr;

    public:
      TimeDependentTranslation(Function<fmatvec::Vec3(double)> *fr_=0) : fr(fr_) { }

      ~TimeDependentTranslation() { delete fr; }

      int getqSize() const { return 0; }

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = (*fr)(t); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = jh; }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = fr->parDer(t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = fr->parDerParDer(t); }
      void updateqd(const fmatvec::VecV &u) { }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(0) = u(0); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(1) = q(0); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(1) = u(1); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInZDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 1;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(2) = q(0); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(2) = u(2); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(1) = q(1); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(0) = u(0); v(1) = u(1); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXZDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(2) = q(1); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(0) = u(0); v(2) = u(1); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYZDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 2;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(1) = q(0); r(2) = q(1); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(1) = u(0); v(2) = u(1); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYZDirection : public Translation {
    public:

      void init();

      int getqSize() const {return 3;}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r(0) = q(0); r(1) = q(1); r(2) = q(2); }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v(0) = u(0); v(1) = u(1); v(2) = u(2); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class LinearTranslation : public Translation {
    private:
      fmatvec::Mat3xV D;

    public:

      LinearTranslation() { }
      LinearTranslation(const fmatvec::Mat3xV &D_) : D(D_) { }

      void init();

      int getqSize() const {return D.cols();}

      void updatePosition(const fmatvec::VecV &q, const double &t) { r = D*q; }
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = J*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

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
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = D*fq->parDer(q); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = D*fq->parDerDirDer(qd,q)*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

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
      void updateVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { v = D*fq->parDer(t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = D*fq->parDer(t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = D*fq->parDerParDer(t); }
      void updateqd(const fmatvec::VecV &u) { }

     void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class Rotation {
    protected:
      fmatvec::RotMat3 A;
      fmatvec::Vec3 om, jh, jb;
      fmatvec::Mat3xV J;
      fmatvec::MatV T;
      fmatvec::VecV qd;
      bool KOSY;

    public:
      Rotation() : KOSY(false) { }
      virtual ~Rotation() { }

      virtual void init();

      virtual int getqSize() const = 0;
      virtual int getuSize() const { return getqSize(); }

      const fmatvec::RotMat3 getOrientation() const {return A;}
      const fmatvec::Vec3 getAngularVelocity() const {return om;}
      const fmatvec::Mat3xV getJacobian() const {return J;}
      const fmatvec::Vec3 getGuidingVelocity() const {return jh;}
      const fmatvec::Vec3 getGyroscopicAcceleration() const {return jb;}
      const fmatvec::MatV getT() const {return T;}
      const fmatvec::VecV getqd() const {return qd;}

      virtual void updateOrientation(const fmatvec::VecV &q, const double &t) { } 
      virtual void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u + jh; }
      virtual void updateJacobian(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { }
      virtual void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { }
      virtual void updateT(const fmatvec::VecV &q, const double &t) { }
      virtual void updateqd(const fmatvec::VecV &u) { qd = T*u; }
      virtual void updateStateDependentVariables(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t);

      void setKOSY(bool KOSY_) { KOSY = KOSY_; }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }
 };

//  class Rotation {
//    protected:
//      Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA;
//      Function<fmatvec::MatV(fmatvec::VecV, double)> *fT;
//      fmatvec::Mat3xV dAdq;
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
//      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = (*fA)(q,t); }
//      void updateJacobian(const fmatvec::VecV &q, const double &t) { dAdq = fA->parDer1(q,t); J = dAdq*T; }
//      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = fA->parDer2(q,t); }
//      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { j = ((fA->parDer1DirDer1(qd,q,t)+fA->parDer1ParDer2(q,t))*T + dAdq*(fT->dirDer1(qd,q,t) + fT->parDer2(q,t)))*u + fA->parDer2DirDer1(qd,q,t) + fA->parDer2ParDer2(q,t); }
//      void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q,t); }
//  };

  class GeneralRotation : public Rotation {
    protected:
      Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA;

    public:
      GeneralRotation(Function<fmatvec::RotMat3(fmatvec::VecV, double)> *fA_=0) : fA(fA_) { }

      ~GeneralRotation() { delete fA; }

      int getqSize() const { return fA->getArg1Size(); }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = (*fA)(q,t); }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = fA->parDer1(q,t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = fA->parDer2(q,t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = (fA->parDer1DirDer1(qd,q,t)+fA->parDer1ParDer2(q,t))*u + fA->parDer2DirDer1(qd,q,t) + fA->parDer2ParDer2(q,t); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }
  };

  class StateDependentRotation : public Rotation {
    protected:
      Function<fmatvec::RotMat3(fmatvec::VecV)> *fA;
      fmatvec::Mat3xV dAdq, dotdAdq;

    public:
      StateDependentRotation(Function<fmatvec::RotMat3(fmatvec::VecV)> *fA_=0) : fA(fA_) { }

      ~StateDependentRotation() { delete fA; }

      int getqSize() const { return fA->getArgSize(); }

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = (*fA)(q); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = fA->parDer(q); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = fA->parDerDirDer(qd,q)*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }
  };

  class RotationAboutXAxis : public Rotation {
    public:
      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om(0) = u(0); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutYAxis : public Rotation {
    public:
      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om(1) = u(0); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutZAxis : public Rotation {
    public:
      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om(2) = u(0); }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXY: public Rotation {
    public:
      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXZ: public Rotation {
    public:
      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesYZ: public Rotation {
    public:
      int getqSize() const {return 2;}

      void updateOrientation(const fmatvec::VecV &q, const double &t);
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t); 
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t); 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutAxesXYZ: public Rotation {
    private:
      FRotationAboutAxesXYZ<fmatvec::VecV> f;
    public:
      int getqSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f(q); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = f.parDer(q); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = f.parDerDirDer(qd,q)*u; } 

      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class RotationAboutFixedAxis : public Rotation {
    private:
      FRotationAboutFixedAxis<fmatvec::VecV> f;

    public:

      RotationAboutFixedAxis(const fmatvec::Vec3 &a=fmatvec::Vec3()) : f(a) { }

      void init();

      int getqSize() const {return 1;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f(q); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

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

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f((*falpha)(q)); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateJacobian(const fmatvec::VecV &q, const double &t) { J = getAxisOfRotation()*falpha->parDer(q); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = getAxisOfRotation()*falpha->parDerDirDer(qd,q)*u; }
      void updateqd(const fmatvec::VecV &u) { qd = u; }

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

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = f((*falpha)(t)); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = getAxisOfRotation()*falpha->parDer(t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = getAxisOfRotation()*falpha->parDer(t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = getAxisOfRotation()*falpha->parDerParDer(t); }
      void updateqd(const fmatvec::VecV &u) { }

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

      int getqSize() const {return 3;}
      int getuSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA(q); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
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

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA((*fangles)(t)); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = fangles->parDer(t); }
      void updateGuidingVelocity(const fmatvec::VecV &q, const double &t) { jh = fangles->parDer(t); }
      void updateGyroscopicAcceleration(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { jb = fangles->parDerParDer(t); }
      void updateqd(const fmatvec::VecV &u) { }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class EulerAngles : public Rotation {
    private:
      FRotationAboutAxesZXZ<fmatvec::VecV> fA;
      Function<fmatvec::MatV(fmatvec::VecV)> *fT;
    public:

      void init();

      int getqSize() const {return 3;}
      int getuSize() const {return 3;}

      void updateOrientation(const fmatvec::VecV &q, const double &t) { A = fA(q); }
      void updateAngularVelocity(const fmatvec::VecV &u, const fmatvec::VecV &q, const double &t) { om = J*u; }
      void updateT(const fmatvec::VecV &q, const double &t) { T = (*fT)(q); }

      void setKOSY(bool KOSY_) { KOSY = KOSY_; }

      void initializeUsingXML(MBXMLUtils::TiXmlElement *element) { }
      MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

}

#endif /* _KINEMATICS_H_ */


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


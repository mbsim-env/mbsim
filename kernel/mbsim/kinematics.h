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

#include "fmatvec.h"
#include "mbsim/utils/function.h"

namespace MBSim {

  /**
   * \brief base class to describe translations along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 Translation inherits Function2 (Martin Foerg)
   */
  class Translation : public Function2<fmatvec::Vec3,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      Translation() {}

      /**
       * \brief destructor
       */
      virtual ~Translation() {}

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return degree of freedom of translation
       */
      virtual int getqSize() const = 0;

      /**
       * \param generalized position
       * \param time
       * \return translational vector as a function of generalized position and time, r=r(q,t)
       */
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }
      /***************************************************/
  };

  class RotationIndependentTranslation : public Translation {
    public:
    /**
       * \brief constructor
       */
      RotationIndependentTranslation() {}

      virtual int getqSize() const { throw; return 0; }
      virtual int getqTSize() const = 0;
      virtual int getuTSize() const = 0;

    protected:
      /**
       * \brief position vector
       */
      fmatvec::Vec3 r;
  };

  class TranslationInXDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInXDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 1;}
      virtual int getuTSize() const {return 1;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(0) = q(0); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInYDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 1;}
      virtual int getuTSize() const {return 1;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(1) = q(0); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInZDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInZDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 1;}
      virtual int getuTSize() const {return 1;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(2) = q(0); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInXYDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 2;}
      virtual int getuTSize() const {return 2;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(0) = q(0); r(1) = q(1); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXZDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInXZDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 2;}
      virtual int getuTSize() const {return 2;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(0) = q(0); r(2) = q(1); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInYZDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInYZDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 2;}
      virtual int getuTSize() const {return 2;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(1) = q(0); r(2) = q(1); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  class TranslationInXYZDirection : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TranslationInXYZDirection() {} 

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 3;}
      virtual int getuTSize() const {return 3;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {r(0) = q(0); r(1) = q(1); r(2) = q(2); return r;}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  /**
   * \brief class to describe translations along a line
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Translation (Martin Foerg)
   */
  class LinearTranslation : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      LinearTranslation() {}

      /**
       * \brief constructor
       * \param independent direction matrix of translation
       */
      LinearTranslation(const fmatvec::Mat3xV &PJT_) { PJT = PJT_; }

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return PJT.cols();}
      virtual int getuTSize() const {return PJT.cols();}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return PJT*q(0,PJT.cols()-1); }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Mat3xV& getTranslationVectors() const { return PJT; }

      /**
       * Set the possible translations vectors. Each column of the matrix
       * is a possible translation vector.
       */
      void setTranslationVectors(const fmatvec::Mat3xV &PJT_) { PJT = PJT_; }
      /***************************************************/

    private:
      /**
       * independent direction matrix of translation
       */
      fmatvec::Mat3xV PJT;
  };

  class TimeDependentLinearTranslation: public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentLinearTranslation() : pos(NULL) {}

      /**
       * \brief constructor
       * \param independent generalized position function
       * \param independent direction matrix of translation
       */
      TimeDependentLinearTranslation(Function1<fmatvec::VecV, double> *pos_, const fmatvec::Mat3xV &PJT_) : PJT(PJT_), pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentLinearTranslation() { delete pos; pos = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqTSize() const {return 0;}
      virtual int getuTSize() const {return 0;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return PJT*(*pos)(t); }
      //virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      Function1<fmatvec::VecV, double>* getTranslationalFunction() { return pos; }
      void setTranslationalFunction(Function1<fmatvec::VecV, double> *pos_) { pos = pos_; }
      const fmatvec::Mat3xV& getTranslationVectors() const { return PJT; }
      void setTranslationVectors(const fmatvec::Mat3xV& PJT_) { PJT = PJT_; }
      /***************************************************/

    private:
      /**
       * independent direction matrix of translation
       */
      fmatvec::Mat3xV PJT;

      /**
       * \brief time dependent generalized position
       */
      Function1<fmatvec::VecV, double> *pos;
  };

  /**
   * \brief class to describe time dependent translations
   * \author Markus Schneider
   * \date 2009-12-21 some adaptations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Translation (Martin Foerg)
   */
  class TimeDependentTranslation : public RotationIndependentTranslation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentTranslation() : pos(NULL) {}

      /**
       * \brief constructor
       * \param independent translation function
       */
      TimeDependentTranslation(Function1<fmatvec::Vec3, double> *pos_) : pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentTranslation() { delete pos; pos = 0; }

      /* INTERFACE OF TRANSLATION */
      virtual int getqTSize() const {return 0;}
      virtual int getuTSize() const {return 0;}
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*pos)(t); }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief set the translation function
       */
      Function1<fmatvec::Vec3, double>* getTranslationFunction() { return pos; }
      void setTranslationFunction(Function1<fmatvec::Vec3, double> *pos_) { pos = pos_; }
      /***************************************************/

    private:
      /**
       * time dependent translation function
       */
      Function1<fmatvec::Vec3, double> *pos;
  };

  class StateDependentTranslation : public Translation {
    public:
      /**
       * \brief constructor
       */
      StateDependentTranslation(int qSize_, Function1<fmatvec::Vec3,fmatvec::Vec> *pos_) : qSize(qSize_), pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~StateDependentTranslation() { delete pos; pos = 0; }

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return degree of freedom of translation
       */
      virtual int getqSize() const {return qSize;}

      /**
       * \param generalized position
       * \param time
       * \return translational vector as a function of generalized position and time, r=r(q,t)
       */
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*pos)(q); }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }

      /* GETTER / SETTER */
      /**
       * \brief set the translation function
       */
      Function1<fmatvec::Vec3,fmatvec::Vec>* getTranslationFunction() { return pos; }
      void setTranslationFunction(Function1<fmatvec::Vec3,fmatvec::Vec> *pos_) { pos = pos_; }
      /***************************************************/

    private:
      int qSize;
      Function1<fmatvec::Vec3,fmatvec::Vec> *pos;
  };

  class GeneralTranslation : public Translation {
    public:
      /**
       * \brief constructor
       */
      GeneralTranslation(int qSize_, Function2<fmatvec::Vec3,fmatvec::Vec,double> *pos_) : qSize(qSize_), pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~GeneralTranslation() { delete pos; pos = 0; }

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return degree of freedom of translation
       */
      virtual int getqSize() const {return qSize;}

      /**
       * \param generalized position
       * \param time
       * \return translational vector as a function of generalized position and time, r=r(q,t)
       */
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*pos)(q,t); }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0; }

      /* GETTER / SETTER */
      /**
       * \brief set the translation function
       */
      Function2<fmatvec::Vec3,fmatvec::Vec,double>* getTranslationFunction() { return pos; }
      void setTranslationFunction(Function2<fmatvec::Vec3,fmatvec::Vec,double> *pos_) { pos = pos_; }
      /***************************************************/

    private:
      int qSize;
      Function2<fmatvec::Vec3,fmatvec::Vec,double> *pos;
  };

  /**
   * \brief base class to describe rotation along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 Rotation inherits Function2 (Martin Foerg)
   */
  class Rotation : public Function2<fmatvec::SqrMat3,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      Rotation() {}

      /**
       * \brief destructor
       */
      virtual ~Rotation() {}

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return degree of freedom of rotation
       */
      virtual int getqSize() const = 0;

       /**
       * \param generalized position
       * \param time
       * \return rotational matrix as a function of generalized position and time, A=A(q,t)
       */
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;

     virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
     virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { return 0;}
      /***************************************************/
  };

  class TranslationIndependentRotation: public Rotation {
    public:

      virtual int getqSize() const { throw; return 0; }
      virtual int getqRSize() const = 0;
      virtual int getuRSize() const = 0;

    protected:
      /**
       * \brief transformation matrix
       */
      fmatvec::SqrMat3 APK;
  };

  /**
   * \brief class to describe rotation about y-axis
   * \author Martin Foerg
   */
  class RotationAboutXAxis: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutXAxis();

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const {return 1;}
      virtual int getuRSize() const {return 1;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about y-axis
   * \author Martin Foerg
   */
  class RotationAboutYAxis: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutYAxis();

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const {return 1;}
      virtual int getuRSize() const {return 1;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about z-axis
   * \author Martin Foerg
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutZAxis: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutZAxis();

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const {return 1;}
      virtual int getuRSize() const {return 1;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about fixed axis
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutFixedAxis: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutFixedAxis() {}

      /**
       * \brief constructor
       * \param axis of rotation
       */
      RotationAboutFixedAxis(const fmatvec::Vec3 &a_) { a = a_; }

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const {return 1;}
      virtual int getuRSize() const {return 1;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Vec3& getAxisOfRotation() const { return a; }
      void setAxisOfRotation(const fmatvec::Vec3& a_) { a = a_; }
      /***************************************************/

    protected:
      /**
       * \brief axis of rotation
       */
      fmatvec::Vec3 a;
  };

  /**
   * \brief class to describe state dependent rotation about fixed axis
   */
  class StateDependentRotationAboutFixedAxis: public Rotation {
    public:

      /**
       * \brief constructor
       */
      StateDependentRotationAboutFixedAxis(int qSize_, Function1<double, fmatvec::Vec> *angle_, const fmatvec::Vec3 &a_) : qSize(qSize_), rot(new RotationAboutFixedAxis(a_)), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~StateDependentRotationAboutFixedAxis() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const {return qSize;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {return (*rot)(fmatvec::Vec(1,fmatvec::INIT,(*angle)(q)),t);} 
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
      /***************************************************/

      /* GETTER / SETTER */
      Function1<double, fmatvec::Vec>* getRotationalFunction() { return angle; }
      void setRotationalFunction(Function1<double, fmatvec::Vec> *angle_) { angle = angle_; }
      const fmatvec::Vec3& getAxisOfRotation() const { return rot->getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec3& a_) { rot->setAxisOfRotation(a_); }
      /***************************************************/

    private:
      int qSize;
      RotationAboutFixedAxis *rot;
      Function1<double, fmatvec::Vec> *angle;
  };

  /**
   * \brief class to describe time dependent rotation about fixed axis
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class TimeDependentRotationAboutFixedAxis: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentRotationAboutFixedAxis() : rot(new RotationAboutFixedAxis()), angle(NULL) {}

      /**
       * \brief constructor
       * \param independent rotation angle function
       * \param axis of rotation
       */
      TimeDependentRotationAboutFixedAxis(Function1<double, double> *angle_, const fmatvec::Vec3 &a_) : rot(new RotationAboutFixedAxis(a_)), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentRotationAboutFixedAxis() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 0; }
      virtual int getuRSize() const {return 0;}
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) {return (*rot)(fmatvec::Vec(1,fmatvec::INIT,(*angle)(t)),t);}
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      Function1<double, double>* getRotationalFunction() { return angle; }
      void setRotationalFunction(Function1<double, double> *angle_) { angle = angle_; }
      const fmatvec::Vec3& getAxisOfRotation() const { return rot->getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec3& a_) { rot->setAxisOfRotation(a_); }
      /***************************************************/

    private:
      /**
       * \brief rotational parametrisation
       */
      RotationAboutFixedAxis *rot;

      /**
       * \brief time dependent rotation angle
       */
      Function1<double, double> *angle;
  };

  /**
   * \brief class to describe rotation about axes x and y with basic rotations interpretated in the current coordinate system
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesXY: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXY() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 2; }
      virtual int getuRSize() const { return 2; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };
  /**
   * \brief class to describe rotation about axes x and z with basic rotations interpretated in the current coordinate system
   * \author Martin Foerg
   */
  class RotationAboutAxesXZ: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXZ() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 2; }
      virtual int getuRSize() const { return 2; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation about axes y and z with basic rotations interpretated in the current coordinate system
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesYZ: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesYZ() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 2; }
      virtual int getuRSize() const { return 2; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by cardan angles
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class CardanAngles: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      CardanAngles() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 3; }
      virtual int getuRSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by Euler angles
   * \author Martin Foerg
   * \date 2010-10-20 first commit (Martin Foerg)
   */
  class EulerAngles: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      EulerAngles() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 3; }
      virtual int getuRSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by cardan angles
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesXYZ: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXYZ() {}

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 3; }
      virtual int getuRSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe time dependent rotation parametrised by Cardan angles
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class TimeDependentCardanAngles: public TranslationIndependentRotation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentCardanAngles() : rot(new CardanAngles()), angle(NULL) {}

      /**
       * \brief constructor
       * \param independent rotation angle function
       */
      TimeDependentCardanAngles(Function1<fmatvec::Vec3, double> *angle_) : rot(new CardanAngles()), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentCardanAngles() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqRSize() const { return 0; }
      virtual int getuRSize() const { return 0; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      Function1<fmatvec::Vec3, double>* getRotationalFunction() { return angle; }
      void setRotationalFunction(Function1<fmatvec::Vec3, double> *angle_) { angle = angle_; }
    private:
      /**
       * \brief rotational parametrisation
       */
      CardanAngles *rot;

      /**
       * \brief time dependent rotation angle
       */
      Function1<fmatvec::Vec3, double> *angle;
  };

  class TMatrix : public Function2<fmatvec::MatV,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      TMatrix() {}

      /**
       * \brief destructor
       */
      virtual ~TMatrix() {}

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return column size of TMatrix
       */
      virtual int getqSize() const = 0;
      virtual int getuSize() const = 0;

      virtual fmatvec::MatV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief base class to describe Jacobians along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2010-05-23 Jacobian inherits Function2 (Martin Foerg)
   */
  class Jacobian : public Function2<fmatvec::Mat3xV,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       */
      Jacobian() {}

      /**
       * \brief destructor
       */
      virtual ~Jacobian() {}

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return column size of Jacobian
       */
      virtual int getuSize() const = 0;

      /**
       * \param generalized position
       * \param time
       * \return Jacobian matrix as a function of generalized position and time,
       * J=J(q,t)
       */
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief class to describe a constant Jacobians
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class ConstantJacobian : public Jacobian {
    public:
      /**
       * \brief constructor
       */
      ConstantJacobian(const fmatvec::Mat &J_) { J = J_; }

      ConstantJacobian() {}

      /**
       * \brief destructor
       */
      virtual ~ConstantJacobian() {}

      /* INTERFACE OF JACOBIAN */
      virtual int getuSize() const { return J.cols(); }
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return J; }
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      /***************************************************/

    private:
      /**
       * \brief constant Jacobian
       */
      fmatvec::Mat3xV J;
  };

  class StateDependentJacobian : public Jacobian {
    public:
      /**
       * \brief constructor
       */
      StateDependentJacobian(int uSize_, Function1<fmatvec::Mat3xV,fmatvec::Vec> *J_) : uSize(uSize_), J(J_) {}

      /**
       * \brief destructor
       */
      virtual ~StateDependentJacobian() { delete J; J = 0; }

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return column size of Jacobian
       */
      virtual int getuSize() const {return uSize;}

      /**
       * \param generalized position
       * \param time
       * \return Jacobian matrix as a function of generalized position and time,
       * J=J(q,t)
       */
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*J)(q); }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};

      /* GETTER / SETTER */
      /**
       * \brief set the Jacobian function
       */
      Function1<fmatvec::Mat3xV,fmatvec::Vec>* getJacobianFunction() { return J; }
      void setJacobianFunction(Function1<fmatvec::Mat3xV,fmatvec::Vec> *J_) { J = J_; }
      /***************************************************/

    private:
      int uSize;
      Function1<fmatvec::Mat3xV,fmatvec::Vec> *J;
  };

  class GeneralJacobian : public Jacobian {
    public:
      /**
       * \brief constructor
       */
      GeneralJacobian(int uSize_, Function2<fmatvec::Mat3xV,fmatvec::Vec,double> *J_) : uSize(uSize_), J(J_) {}

      /**
       * \brief destructor
       */
      virtual ~GeneralJacobian() { delete J; J = 0; }

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \return column size of Jacobian
       */
      virtual int getuSize() const {return uSize;}

      /**
       * \param generalized position
       * \param time
       * \return Jacobian matrix as a function of generalized position and time,
       * J=J(q,t)
       */
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*J)(q,t); }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {};

      /* GETTER / SETTER */
      /**
       * \brief set the Jacobian function
       */
      Function2<fmatvec::Mat3xV,fmatvec::Vec,double>* getJacobianFunction() { return J; }
      void setJacobianFunction(Function2<fmatvec::Mat3xV,fmatvec::Vec,double> *J_) { J = J_; }
      /***************************************************/

    private:
      int uSize;
      Function2<fmatvec::Mat3xV,fmatvec::Vec,double> *J;
  };

  /**
   * \brief Jacobian for rotation about axes x and y
   * \author Martin Foerg
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class JRotationAboutAxesXY : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesXY(int uSize_) : uSize(uSize_), J(uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV J;
  };

  /**
   * \brief Jacobian for rotation about axes x and z
   * \author Martin Foerg
   */
  class JRotationAboutAxesXZ : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesXZ(int uSize_) : uSize(uSize_), J(uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV J;
  };

  /**
   * \brief Jacobian for rotation about axes y and z
   * \author Martin Foerg
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class JRotationAboutAxesYZ : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesYZ(int uSize_) : uSize(uSize_), J(uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV J;
  };

  /**
   * \brief Jacobian for rotation about axes x and y
   * \author Martin Foerg
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class JRotationAboutAxesXYZ : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesXYZ(int uSize_) : uSize(uSize_), J(uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV J;
  };

  /**
   * \brief standard parametrisation with angular velocity in reference system yields time-dependent mass matrix
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class TCardanAngles : public TMatrix {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TCardanAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      /* INTERFACE OF JACOBIAN */
      int getqSize() const { return qSize; }
      int getuSize() const { return uSize; }
      virtual fmatvec::MatV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::MatV T;
  };

  /**
   * \brief standard parametrisation with angular velocity in reference system yields time-dependent mass matrix
   * \author Martin Foerg
   * \date 2010-10-20 first commit (Martin Foerg)
   */
  class TEulerAngles : public TMatrix {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TEulerAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {
	int iq = qSize-1;
	int iu = uSize-1;
	T(iq-2,iu) = 1;
	T(iq,iu) = 0;
      }

      /* INTERFACE OF JACOBIAN */
      int getqSize() const { return qSize; }
      int getuSize() const { return uSize; }
      virtual fmatvec::MatV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::MatV T;
  };

  /**
   * \brief alternative parametrisation with angular velocity in body frame yields constant mass matrix for absolute coordinates
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Jacobian (Martin Foerg)
   */
  class TCardanAngles2 : public TMatrix {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TCardanAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      /* INTERFACE OF JACOBIAN */
      int getqSize() const { return qSize; }
      int getuSize() const { return uSize; }
      virtual fmatvec::MatV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::MatV T;
  };

  /**
   * \brief alternative parametrisation with angular velocity in body frame yields constant mass matrix for absolute coordinates
   * \author Martin Foerg
   * \date 2010-10-20 first commit (Martin Foerg)
   */
  class TEulerAngles2 : public TMatrix {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TEulerAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {
      }

      /* INTERFACE OF JACOBIAN */
      int getqSize() const { return qSize; }
      int getuSize() const { return uSize; }
      virtual fmatvec::MatV operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::MatV T;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes x and y
   * \author Martin Foerg
   */
  class JdRotationAboutAxesXY : public Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXY(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes x and z
   * \author Martin Foerg
   */
  class JdRotationAboutAxesXZ : public Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXZ(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes y and z
   * \author Martin Foerg
   */
  class JdRotationAboutAxesYZ : public Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesYZ(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes x and y
   * \author Martin Foerg
   */
  class JdRotationAboutAxesXYZ : public Function3<fmatvec::Mat3xV,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXYZ(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3xV operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3xV Jd;
  };

}

#endif /* _KINEMATICS_H_ */


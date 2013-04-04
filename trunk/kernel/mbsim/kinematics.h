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

      virtual void initializeUsingXML(TiXmlElement *element) {}
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) { return 0; }
      /***************************************************/
  };

  /**
   * \brief class to describe translations along a line
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Translation (Martin Foerg)
   */
  class LinearTranslation : public Translation {
    public:
      /**
       * \brief constructor
       */
      LinearTranslation() {}

      /**
       * \brief constructor
       * \param independent direction matrix of translation
       */
      LinearTranslation(const fmatvec::Mat3V &PJT_) { PJT = PJT_; }

      /* INTERFACE OF TRANSLATION */
      virtual int getqSize() const { throw; return 0; }
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return PJT*q(0,PJT.cols()-1); }
      virtual void initializeUsingXML(TiXmlElement *element);
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Mat3V& getTranslationVectors() const { return PJT; }

      /**
       * Set the possible translations vectors. Each column of the matrix
       * is a possible translation vector.
       */
      void setTranslationVectors(const fmatvec::Mat3V &PJT_) { PJT = PJT_; }
      /***************************************************/

    private:
      /**
       * independent direction matrix of translation
       */
      fmatvec::Mat3V PJT;
  };

  /**
   * \brief class to describe time dependent translations
   * \author Markus Schneider
   * \date 2009-12-21 some adaptations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Translation (Martin Foerg)
   */
  class TimeDependentTranslation : public Translation {
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
      virtual int getqSize() const { return 0; }
      virtual fmatvec::Vec3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*pos)(t); }
      virtual void initializeUsingXML(TiXmlElement *element);
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

  class GeneralTranslation : public Translation {
    public:
      /**
       * \brief constructor
       */
      GeneralTranslation(int qSize_, Function2<fmatvec::Vec3,fmatvec::Vec,double> *pos_) : qSize(qSize_), pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~GeneralTranslation() {}

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

      virtual void initializeUsingXML(TiXmlElement *element) {}
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) { return 0; }
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

     virtual void initializeUsingXML(TiXmlElement *element) {}
     virtual TiXmlElement* writeXMLFile(TiXmlNode *parent) { return 0;}
      /***************************************************/
  };

  class RotationAboutOneAxis: public Rotation {
    public:
    /**
       * \brief constructor
       */
      RotationAboutOneAxis() {}

      virtual int getqSize() const { throw; return 0; }

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
  class RotationAboutXAxis: public RotationAboutOneAxis {
    public:
      /**
       * \brief constructor
       */
      RotationAboutXAxis();

      /* INTERFACE OF ROTATION */
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {}
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about y-axis
   * \author Martin Foerg
   */
  class RotationAboutYAxis: public RotationAboutOneAxis {
    public:
      /**
       * \brief constructor
       */
      RotationAboutYAxis();

      /* INTERFACE OF ROTATION */
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {}
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about z-axis
   * \author Martin Foerg
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutZAxis: public RotationAboutOneAxis {
    public:
      /**
       * \brief constructor
       */
      RotationAboutZAxis();

      /* INTERFACE OF ROTATION */
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {}
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
  };

  /**
   * \brief class to describe rotation about fixed axis
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutFixedAxis: public RotationAboutOneAxis {
    public:
      /**
       * \brief constructor
       */
      RotationAboutFixedAxis() : RotationAboutOneAxis() {}

      /**
       * \brief constructor
       * \param axis of rotation
       */
      RotationAboutFixedAxis(const fmatvec::Vec3 &a_) : RotationAboutOneAxis() { a = a_; }

      /* INTERFACE OF ROTATION */
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element);
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
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
   * \brief class to describe time dependent rotation about fixed axis
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class TimeDependentRotationAboutFixedAxis: public Rotation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentRotationAboutFixedAxis() : Rotation(), rot(new RotationAboutFixedAxis()), angle(NULL) {}

      /**
       * \brief constructor
       * \param independent rotation angle function
       * \param axis of rotation
       */
      TimeDependentRotationAboutFixedAxis(Function1<double, double> *angle_, const fmatvec::Vec3 &a_) : Rotation(), rot(new RotationAboutFixedAxis(a_)), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentRotationAboutFixedAxis() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 0; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element);
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

  class RotationAboutTwoAxes: public Rotation {
    public:
    /**
       * \brief constructor
       */
      RotationAboutTwoAxes() {}

      virtual int getqSize() const { throw; return 0; }

    protected:
      /**
       * \brief transformation matrix
       */
      fmatvec::SqrMat3 APK;
  };

  /**
   * \brief class to describe rotation about axes x and y with basic rotations interpretated in the current coordinate system
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesXY: public RotationAboutTwoAxes {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXY() : RotationAboutTwoAxes() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 2; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation about axes y and z with basic rotations interpretated in the current coordinate system
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesYZ: public RotationAboutTwoAxes {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesYZ() : RotationAboutTwoAxes() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 2; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/
  };

 class RotationAboutThreeAxes: public Rotation {
    public:
    /**
       * \brief constructor
       */
      RotationAboutThreeAxes() {}

      virtual int getqSize() const { throw; return 0; }

    protected:
      /**
       * \brief transformation matrix
       */
      fmatvec::SqrMat3 APK;
  };

  /**
   * \brief class to describe rotation parametrised by cardan angles
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class CardanAngles: public RotationAboutThreeAxes {
    public:
      /**
       * \brief constructor
       */
      CardanAngles() : RotationAboutThreeAxes() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by Euler angles
   * \author Martin Foerg
   * \date 2010-10-20 first commit (Martin Foerg)
   */
  class EulerAngles: public RotationAboutThreeAxes {
    public:
      /**
       * \brief constructor
       */
      EulerAngles() : RotationAboutThreeAxes() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by cardan angles
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class RotationAboutAxesXYZ: public RotationAboutThreeAxes {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXYZ() : RotationAboutThreeAxes() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      virtual TiXmlElement* writeXMLFile(TiXmlNode *parent);
      /***************************************************/
  };

  /**
   * \brief class to describe time dependent rotation parametrised by Cardan angles
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
   * \date 2010-05-23 update according to change in Rotation (Martin Foerg)
   */
  class TimeDependentCardanAngles: public RotationAboutThreeAxes {
    public:
      /**
       * \brief constructor
       */
      TimeDependentCardanAngles() : RotationAboutThreeAxes(), rot(new CardanAngles()), angle(NULL) {}

      /**
       * \brief constructor
       * \param independent rotation angle function
       */
      TimeDependentCardanAngles(Function1<fmatvec::Vec3, double> *angle_) : RotationAboutThreeAxes(), rot(new CardanAngles()), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentCardanAngles() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 0; }
      virtual fmatvec::SqrMat3 operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

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

      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief base class to describe Jacobians along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2010-05-23 Jacobian inherits Function2 (Martin Foerg)
   */
  class Jacobian : public Function2<fmatvec::Mat3V,fmatvec::Vec,double> {
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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;

      virtual void initializeUsingXML(TiXmlElement *element) {};
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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return J; }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

    private:
      /**
       * \brief constant Jacobian
       */
      fmatvec::Mat3V J;
  };

  class GeneralJacobian : public Jacobian {
    public:
      /**
       * \brief constructor
       */
      GeneralJacobian(int uSize_, Function2<fmatvec::Mat3V,fmatvec::Vec,double> *J_) : uSize(uSize_), J(J_) {}

      /**
       * \brief destructor
       */
      virtual ~GeneralJacobian() {}

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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return (*J)(q,t); }

      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/

    private:
      int uSize;
      Function2<fmatvec::Mat3V,fmatvec::Vec,double> *J;
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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V J;
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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V J;
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
      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &q, const double &t, const void * =NULL);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V J;
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
  class JdRotationAboutAxesXY : public Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXY(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes y and z
   * \author Martin Foerg
   */
  class JdRotationAboutAxesYZ : public Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesYZ(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes x and y
   * \author Martin Foerg
   */
  class JdRotationAboutAxesXYZ : public Function3<fmatvec::Mat3V,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXYZ(int uSize_) : uSize(uSize_), Jd(uSize) {}

      virtual fmatvec::Mat3V operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat3V Jd;
  };

////   class Kinematics {
////     public:
////
////       Kinematics();
////
////       int getqSize() const {return qSize;}
////       int getuSize(int j=0) const {return uSize[j];}
////
////       //void update(const fmatvec::Vec& uRel, const fmatvec::Vec& qRel, double t);
////       //void updateqdRel(const fmatvec::Vec& uRel) { qdRel = T*uRel; }
////       void updateT(const fmatvec::Vec& qRel, double t) { if(fT) T = (*fT)(qRel,t); }
////       void updatePrPK(const fmatvec::Vec& qRel, double t) { if(fPrPK) PrPK = (*fPrPK)(qRel,t); }
////       void updateAPK(const fmatvec::Vec& qRel, double t) { if(fAPK) APK = (*fAPK)(qRel,t); }
////       void updatePJT(const fmatvec::Vec& qRel, double t) { if(fPJT) PJT[0] = (*fPJT)(qRel,t); }
////       void updatePJR(const fmatvec::Vec& qRel, double t) { if(fPJR) PJR[0] = (*fPJR)(qRel,t); }
////       void updatePdJT(const fmatvec::Vec& qdRel, const fmatvec::Vec& qRel, double t) { if(fPdJT) PdJT = (*fPdJT)(qdRel,qRel,t); }
////       void updatePdJR(const fmatvec::Vec& qdRel, const fmatvec::Vec& qRel, double t) { if(fPdJR) PdJR = (*fPdJR)(qdRel,qRel,t); }
////       void updatePjT(double t) { if(fPjT) PjT = (*fPjT)(t); }
////       void updatePjR(double t) { if(fPjR) PjR = (*fPjR)(t); }
////       void updatePdjT(double t) { if(fPdjT) PdjT = (*fPdjT)(t); }
////       void updatePdjR(double t) { if(fPdjR) PdjR = (*fPdjR)(t); }
////       virtual void calcqSize();
////       virtual void calcuSize(int j=0);
////       virtual void init(InitStage stage);
////
////     public:
////       int qSize, uSize[2];
////
////       fmatvec::Mat PJT[2], PJR[2], PdJT, PdJR;
////
////       fmatvec::Vec PjT, PjR, PdjT, PdjR;
////       fmatvec::SqrMat APK;
////       fmatvec::Vec PrPK;
////       fmatvec::Mat T;
////       //fmatvec::Vec qdRel;
////
////       Jacobian *fT;
////
////       Translation *fPrPK;
////
////       Rotation *fAPK;
////
////       Jacobian *fPJT;
////       Jacobian *fPJR;
////
////       Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double> *fPdJT;
////       Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double> *fPdJR;
////
////       Function1<fmatvec::Vec,double> *fPjT;
////       Function1<fmatvec::Vec,double> *fPjR;
////
////       Function1<fmatvec::Vec,double> *fPdjT;
////       Function1<fmatvec::Vec,double> *fPdjR;
////
////       void setTranslation(Translation* fPrPK_) { fPrPK = fPrPK_; }
////       void setRotation(Rotation* fAPK_)        { fAPK  = fAPK_;  }
////       Translation* getTranslation()            { return fPrPK;   }
////       Rotation*    getRotation()               { return fAPK;    }
////       void setJacobianOfTranslation(Jacobian* fPJT_) { fPJT = fPJT_; }
////       void setJacobianOfRotation(Jacobian* fPJR_)    { fPJR = fPJR_; }
////       void setDerivativeOfJacobianOfTranslation(Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>* fPdJT_) { fPdJT = fPdJT_;}
////       void setDerivativeOfJacobianOfRotation(Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>* fPdJR_) { fPdJR = fPdJR_;}
////       void setGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPjT_) { fPjT = fPjT_;}
////       void setGuidingVelocityOfRotation(Function1<fmatvec::Vec,double>* fPjR_) { fPjR = fPjR_;}
////       void setDerivativeOfGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPdjT_) { fPdjT = fPdjT_;}
////       void setDerivativeOfGuidingVelocityOfRotation(Function1<fmatvec::Vec,double>* fPdjR_) { fPdjR = fPdjR_;}
////       fmatvec::Mat& getT() {return T;}
////       fmatvec::Mat& getPJT(int i=0) {return PJT[i];}
////       fmatvec::Mat& getPJR(int i=0) {return PJR[i];}
////       fmatvec::Mat& getPdJT() {return PdJT;}
////       fmatvec::Mat& getPdJR() {return PdJR;}
////       fmatvec::Vec& getPjT() {return PjT;}
////       fmatvec::Vec& getPjR() {return PjR;}
////       fmatvec::Vec& getPdjT() {return PdjT;}
////       fmatvec::Vec& getPdjR() {return PdjR;}
////       fmatvec::Vec& getPrPK() {return PrPK;}
////       fmatvec::SqrMat& getAPK() {return APK;}
////
////
////   };
////
//// //  class KinematicsTranslation {
//// //    public:
//// //      KinematicsTranslation();
//// //      virtual ~KinematicsTranslation() {}
//// //      //void setTranslation(Translation* fPrPK_) { fPrPK = fPrPK_; }
//// //      //Translation* getTranslation()            { return fPrPK;   }
//// //      //void setJacobianOfTranslation(Jacobian* fPJT_) { fPJT = fPJT_; }
//// //      //void setDerivativeOfJacobianOfTranslation(Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>* fPdJT_) { fPdJT = fPdJT_;}
//// //      //void setGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPjT_) { fPjT = fPjT_;}
//// //      //void setDerivativeOfGuidingVelocityOfTranslation(Function1<fmatvec::Vec,double>* fPdjT_) { fPdjT = fPdjT_;}
//// //      virtual void update(const fmatvec::Vec& qdRel, const fmatvec::Vec& qRel, double t);
//// //      fmatvec::Mat& getTranslation() { return PrPK; }
//// //      fmatvec::Mat& getJacobianOfTranslation() { return PJT; }
//// //      fmatvec::Mat& getDerivativeOfJacobianOfTranslation() { return PdJT; }
//// //      fmatvec::Vec& getGuidingVelocityOfTranslation() { return PjT;}
//// //      fmatvec::Vec& getDerivativeOfGuidingVelocityOfTranslation() { return PdjT;}
//// //      const fmatvec::Mat& getTranslation() const { return PrPK; }
//// //      const fmatvec::Mat& getJacobianOfTranslation() const { return PJT; }
//// //      const fmatvec::Mat& getDerivativeOfJacobianOfTranslation() const { return PdJT; }
//// //      const fmatvec::Vec& getGuidingVelocityOfTranslation() const { return PjT;}
//// //      const fmatvec::Vec& getDerivativeOfGuidingVelocityOfTranslation() const { return PdjT;}
//// //    protected:
//// //      virtual fmatvec::Vec fPrPK(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Mat fPJT(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Mat fPdJT(const fmatvec::Vec &qd, const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Vec fPjT(const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Vec fPdjT(const double &t, const void * =NULL) = 0;
//// //      //Translation *fPrPK;
//// //      //Jacobian *fPJT;
//// //      //Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double> *fPdJT;
//// //      //Function1<fmatvec::Vec,double> *fPjT;
//// //      //Function1<fmatvec::Vec,double> *fPdjT;
//// //      fmatvec::Mat PJT, PdJT;
//// //      fmatvec::Vec PrPK, PjT, PdjT;
//// //
//// //  };
//// //
//// //  class KinematicsRotation {
//// //    public:
//// //      KinematicsRotation();
//// //      virtual ~KinematicsRotation() {}
//// //      virtual void update(const fmatvec::Vec& qdRel, const fmatvec::Vec& qRel, double t);
//// //      fmatvec::Mat& getRotation() { return APK; }
//// //      fmatvec::Mat& getJacobianOfRotation() { return PJR; }
//// //      fmatvec::Mat& getDerivativeOfJacobianOfRotation() { return PdJR; }
//// //      fmatvec::Vec& getGuidingVelocityOfRotation() { return PjR;}
//// //      fmatvec::Vec& getDerivativeOfGuidingVelocityOfRotation() { return PdjR;}
//// //      const fmatvec::Mat& getRotation() const { return APK; }
//// //      const fmatvec::Mat& getJacobianOfRotation() const { return PJR; }
//// //      const fmatvec::Mat& getDerivativeOfJacobianOfRotation() const { return PdJR; }
//// //      const fmatvec::Vec& getGuidingVelocityOfRotation() const { return PjR;}
//// //      const fmatvec::Vec& getDerivativeOfGuidingVelocityOfRotation() const { return PdjR;}
//// //    protected:
//// //      virtual fmatvec::SqrMat fAPK(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Mat fPJR(const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Mat fPdJR(const fmatvec::Vec &qd, const fmatvec::Vec &q, const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Vec fPjR(const double &t, const void * =NULL) = 0;
//// //      virtual fmatvec::Vec fPdjR(const double &t, const void * =NULL) = 0;
//// //      fmatvec::Mat PJR, PdJR;
//// //      fmatvec::SqrMat APK;
//// //      fmatvec::Vec PjR, PdjR;
//// //
//// //  };
//// //
//// //  class Kinematics {
//// //    public:
//// //
//// //      int getqSize() const {return qSize;}
//// //      int getuSize() const {return uSize;}
//// //
//// //      virtual void update(const fmatvec::Vec& uRel, const fmatvec::Vec& qRel, double t);
//// //
//// //    protected:
//// //      int qSize, uSize;
//// //
//// //      Jacobian *fT;
//// //      KinematicsTranslation *translation;
//// //      KinematicsRotation *rotation;
//// //      fmatvec::Mat T;
//// //  };
//// //
//// //  class KinematicsLinearTranslation : public KinematicsTranslation {
//// //    public:
//// //      KinematicsLinearTranslation(const fmatvec::Mat &PJT_) : KinematicsTranslation() { PJT = PJT_; }
//// //
//// //      /* INTERFACE OF TRANSLATION */
//// //      //virtual fmatvec::Vec operator()(const fmatvec::Vec &q, const double &t, const void * =NULL) { return PJT*q(0,PJT.cols()-1); }
//// //      //virtual void initializeUsingXML(TiXmlElement *element);
//// //      /***************************************************/
//// //
//// //      /* GETTER / SETTER */
//// //      const fmatvec::Mat& getTranslationVectors() const { return PJT; }
//// //
//// //      /**
//// //       * Set the posible translations vectors. Each column of the matrix
//// //       * is a posible translation vector.
//// //       */
//// //      void setTranslationVectors(const fmatvec::Mat &PJT_) { PJT = PJT_; }
//// //      /***************************************************/
//// //
//// //    protected:
//// //      virtual fmatvec::Vec fPrPK(const fmatvec::Vec &q, const double &t, const void * =NULL) { return PJT*q(0,PJT.cols()-1); }
//// //      virtual fmatvec::Mat fPJT(const fmatvec::Vec &q, const double &t, const void * =NULL) { return fmatvec::Mat(3,q.size()); }
//// //      virtual fmatvec::Mat fPdJT(const fmatvec::Vec &u, const fmatvec::Vec &q, const double &t, const void * =NULL) { return fmatvec::Mat(3,q.size()); }
//// //      virtual fmatvec::Vec fPjT(const double &t, const void * =NULL) { return fmatvec::Vec(3); }
//// //      virtual fmatvec::Vec fPdjT(const double &t, const void * =NULL) { return fmatvec::Vec(3); }
//// //    private:
//// //      /**
//// //       * independent direction matrix of translation
//// //       */
//// //      fmatvec::Mat PJT;
//// //  };
//// //
//// //  class KinematicsRotationAboutFixedAxis : public KinematicsRotation {
//// //    public:
//// //      KinematicsRotationAboutFixedAxis(const fmatvec::Vec &a_) : KinematicsRotation() { a = a_; }
//// //
//// //      const fmatvec::Vec& getAxisOfRotation() const { return a; }
//// //      void setAxisOfRotation(const fmatvec::Vec& a_) { a = a_; }
//// //    protected:
//// //      virtual fmatvec::SqrMat fAPK(const fmatvec::Vec &q, const double &t, const void * =NULL);
//// //      virtual fmatvec::Mat fPJR(const fmatvec::Vec &q, const double &t, const void * =NULL) { return fmatvec::Mat(3,q.size()); }
//// //
//// //      virtual fmatvec::Mat fPdJR(const fmatvec::Vec &u, const fmatvec::Vec &q, const double &t, const void * =NULL) { return fmatvec::Mat(3,q.size()); }
//// //
//// //      virtual fmatvec::Vec fPjR(const double &t, const void * =NULL) { return fmatvec::Vec(3); }
//// //      virtual fmatvec::Vec fPdjR(const double &t, const void * =NULL) { return fmatvec::Vec(3); }
//// //      fmatvec::Mat PJR, PdJR;
//// //      fmatvec::SqrMat APK;
//// //      fmatvec::Vec PjR, PdjR;
//// //
//// //    private:
//// //      fmatvec::Vec a;
//// //  };

}

#endif /* _KINEMATICS_H_ */


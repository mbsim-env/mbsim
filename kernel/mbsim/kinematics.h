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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "fmatvec.h"
#include <fstream>
#include "mbsim/utils/function.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

namespace MBSim {

  /**
   * \brief base class to describe translations along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class Translation {
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
       * \param functional coordinates 
       * \param time
       * \return translational vector as a function of its degree of freedom
       */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &q, double t) = 0; 

      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief class to describe translations along a line
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
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
      LinearTranslation(const fmatvec::Mat &PJT_) { PJT = PJT_; } 

      /* INTERFACE OF TRANSLATION */
      virtual int getqSize() const { return PJT.cols(); }
      virtual fmatvec::Vec operator()(const fmatvec::Vec &q, double t) {
        return PJT*q(0,PJT.cols()-1);
      }; 
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Mat& getTranslationVectors() const { return PJT; }

      /**
       * Set the posible translations vectors. Each column of the matrix
       * is a posible translation vector.
       */
      void setTranslationVectors(const fmatvec::Mat &PJT_) { PJT = PJT_; }
      /***************************************************/

    private:
      /**
       * independent direction matrix of translation
       */
      fmatvec::Mat PJT;
  };

  /**
   * \brief class to describe time dependent translations
   * \author Markus Schneider
   * \date 2009-12-21 some adaptations (Thorsten Schindler)
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
      TimeDependentTranslation(Function1<fmatvec::Vec, double> *pos_) : pos(pos_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentTranslation() { delete pos; pos = 0; } 

      /* INTERFACE OF TRANSLATION */
      virtual int getqSize() const { return 0; }
      virtual fmatvec::Vec operator()(const fmatvec::Vec &q, double t) { return (*pos)(t); }
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      /**
       * \brief set the translation function
       */
      void setTranslationFunction(Function1<fmatvec::Vec, double> *pos_) { pos = pos_; }
      /***************************************************/

    private:
      /**
       * time dependent translation function
       */
      Function1<fmatvec::Vec, double> *pos;
  };

  /**
   * \brief base class to describe rotation along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class Rotation {
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
       * \param functional coordinates 
       * \param time
       * \return rotational matrix as a function of its degree of freedom
       */
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t) = 0; 

      virtual void initializeUsingXML(TiXmlElement *element) {}
      /***************************************************/
  };

  /**
   * \brief class to describe rotation about fixed axis
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class RotationAboutFixedAxis: public Rotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutFixedAxis() : Rotation(), a(3) {}

      /**
       * \brief constructor
       * \param axis of rotation
       */
      RotationAboutFixedAxis(const fmatvec::Vec &a_) : Rotation() { a = a_; } 

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 1; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Vec& getAxisOfRotation() const { return a; }
      void setAxisOfRotation(const fmatvec::Vec& a_) { a = a_; }
      /***************************************************/

    protected:
      /**
       * \brief axis of rotation
       */
      fmatvec::Vec a;
  };

  /**
   * \brief class to describe time dependent rotation about fixed axis
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
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
      TimeDependentRotationAboutFixedAxis(Function1<double, double> *angle_, const fmatvec::Vec &a_) : Rotation(), rot(new RotationAboutFixedAxis(a_)), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentRotationAboutFixedAxis() { delete rot; rot = 0; delete angle; angle = 0; }


      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 0; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

      /* GETTER / SETTER */
      void setRotationalFunction(Function1<double, double> *angle_) { angle = angle_; }
      const fmatvec::Vec& getAxisOfRotation() const { return rot->getAxisOfRotation(); }
      void setAxisOfRotation(const fmatvec::Vec& a_) { rot->setAxisOfRotation(a_); }
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
   * \brief class to describe rotation about axes x and y
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   */
  class RotationAboutAxesXY: public Rotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesXY() : Rotation() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 2; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief class to describe rotation about axes y and z
   * \author Martin Foerg
   * \date 2009-12-21 some localisations (Thorsten Schindler)
   */
  class RotationAboutAxesYZ: public Rotation {
    public:
      /**
       * \brief constructor
       */
      RotationAboutAxesYZ() : Rotation() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 2; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief class to describe rotation parametrised by cardan angles
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class CardanAngles: public Rotation {
    public:
      /**
       * \brief constructor
       */
      CardanAngles() : Rotation() {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief class to describe time dependent rotation parametrised by Cardan angles
   * \author Thorsten Schindler
   * \date 2009-12-21 initial commit (Thorsten Schindler)
   * \date 2009-12-22 should be a rotation because otherwise it has some dof (Thorsten Schindler)
   */
  class TimeDependentCardanAngles: public Rotation {
    public:
      /**
       * \brief constructor
       */
      TimeDependentCardanAngles() : Rotation(), rot(new CardanAngles()), angle(NULL) {}

      /**
       * \brief constructor
       * \param independent rotation angle function
       */
      TimeDependentCardanAngles(Function1<fmatvec::Vec, double> *angle_) : Rotation(), rot(new CardanAngles()), angle(angle_) {}

      /**
       * \brief destructor
       */
      virtual ~TimeDependentCardanAngles() { delete rot; rot = 0; delete angle; angle = 0; }

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 0; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
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
      Function1<fmatvec::Vec, double> *angle;
  };

  /**
   * \brief base class to describe Jacobians along a path
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Jacobian {
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
       * \param functional coordinates 
       * \param time
       * \return Jacobian matrix as a function of its degree of freedom
       */
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t) = 0;

      virtual void initializeUsingXML(TiXmlElement *element) {};
      /***************************************************/
  };

  /**
   * \brief class to describe a constant Jacobians
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
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
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t) { return J; } 
      virtual void initializeUsingXML(TiXmlElement *element);
      /***************************************************/

    private:
      /**
       * \brief constant Jacobian
       */
      fmatvec::Mat J;
  };

  /**
   * \brief Jacobian for rotation about axes x and y
   * \author Martin Foerg
   */
  class JRotationAboutAxesXY : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesXY(int uSize_) : uSize(uSize_), J(3,uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat J;
  };

  /**
   * \brief Jacobian for rotation about axes y and z
   * \author Martin Foerg
   */
  class JRotationAboutAxesYZ : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JRotationAboutAxesYZ(int uSize_) : uSize(uSize_), J(3,uSize) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat J;
  };

  /**
   * \brief standard parametrisation with angular velocity in reference system yields time-dependent mass matrix
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class TCardanAngles : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TCardanAngles(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat T;
  };

  /**
   * \brief alternative parametrisation with angular velocity in body frame yields constant mass matrix for absolute coordinates
   * \author Martin Foerg
   * \date 2009-04-08 some comments (Thorsten Schindler)
   */
  class TCardanAngles2 : public Jacobian {
    public:
      /**
       * \brief constructor
       * \param size of positions
       * \param size of velocities
       */
      TCardanAngles2(int qSize_, int uSize_) : qSize(qSize_), uSize(uSize_), T(qSize,uSize,fmatvec::EYE) {}

      /* INTERFACE OF JACOBIAN */
      int getuSize() const { return uSize; }
      virtual fmatvec::Mat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

    private:
      /**
       * \brief size of positions and velocities
       */
      int qSize, uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat T;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes x and y
   * \author Martin Foerg
   */
  class JdRotationAboutAxesXY : public Function3<fmatvec::Mat,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesXY(int uSize_) : uSize(uSize_), Jd(3,uSize) {}

      virtual fmatvec::Mat operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat Jd;
  };

  /**
   * \brief derivative of Jacobian for rotation about axes y and z
   * \author Martin Foerg
   */
  class JdRotationAboutAxesYZ : public Function3<fmatvec::Mat,fmatvec::Vec,fmatvec::Vec,double> {
    public:
      /**
       * \brief constructor
       * \param size of generalized velocity vector
       */
      JdRotationAboutAxesYZ(int uSize_) : uSize(uSize_), Jd(3,uSize) {}

      virtual fmatvec::Mat operator()(const fmatvec::Vec &qd, const fmatvec::Vec& q, const double& t, const void*);

    private:
      /**
       * \brief size of positions and velocities
       */
      int uSize;

      /**
       * \brief linear relation between differentiated positions and velocities
       */
      fmatvec::Mat Jd;
  };

}

#endif /* _KINEMATICS_H_ */


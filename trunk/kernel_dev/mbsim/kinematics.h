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
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element) {}
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
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Mat& getTranslationVectors() const { return PJT; }

      /**
       * Set the posible translations vectors. Each column of the matrix
       * is a posible translation vector.
       */
      void setTranslationVectors(const fmatvec::Mat &PJT_) { PJT = PJT_; }
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element);
    
    private:
      /**
       * independent direction matrix of translation
       */
      fmatvec::Mat PJT;
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
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element) {}
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
      RotationAboutFixedAxis() : APK(3), a(3) {}

      /**
       * \brief constructor
       * \param axis of rotation
       */
      RotationAboutFixedAxis(const fmatvec::Vec &a_) : APK(3) { a = a_; } 

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 1; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

      /* GETTER / SETTER */
      const fmatvec::Vec& getAxisOfRotation() const { return a; }
      void setAxisOfRotation(const fmatvec::Vec& a_) { a = a_; }
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element);
        
        private:
      /**
       * \brief rotational matrix
       */
      fmatvec::SqrMat APK;

      /**
       * \brief axis of rotation
       */
      fmatvec::Vec a;
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
      CardanAngles() : APK(3) {}

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      virtual fmatvec::SqrMat operator()(const fmatvec::Vec &q, double t);
      /***************************************************/

    private:
      /**
       * \brief rotational matrix
       */
      fmatvec::SqrMat APK;
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
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element) {};
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
      /***************************************************/

      virtual void initializeUsingXML(TiXmlElement *element);
    
    private:
      /**
       * \brief constant Jacobian
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

}

#endif


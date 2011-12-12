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

#ifndef _EXTRADYNAMIC_INTERFACE_H_
#define _EXTRADYNAMIC_INTERFACE_H_

namespace MBSim {

  /*!
   * \brief interface for dynamic systems of the form \f$\dot{x}=f\left(x\right)\f$
   * \author Thorsten Schindler
   * \date 2009-04-06 initial commit (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   */
  class ExtraDynamicInterface {
    public:
      /*!
       * \brief constructor 
       */
      ExtraDynamicInterface() {}

      /*!
       * \brief destructor
       */
      virtual ~ExtraDynamicInterface() {}

      /*!
       * \brief update order one parameter increment
       * \param simulation time
       * \param simulation step size
       */
      virtual void updatedx(double t, double dt) = 0;

      /*!
       * \brief update differentiated order one parameter
       * \param simulation time
       */
      virtual void updatexd(double t) = 0;

      /**
       * \brief calculates size of order one parameters
       */
      virtual void calcxSize() = 0;

      /**
       * \return order one parameters
       */
      virtual const fmatvec::Vec& getx() const = 0;

      /**
       * \return order one parameters
       */
      virtual fmatvec::Vec& getx() = 0;

      /**
       * \param order one parameter index
       */
      virtual void setxInd(int xInd_) = 0;

      /**
       * \return size of order one parameters
       */
      virtual int getxSize() const = 0;

      /**
       * \brief references to order one parameter of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updatexRef(const fmatvec::Vec& ref) = 0;

      /**
       * \brief references to order one parameter derivatives of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updatexdRef(const fmatvec::Vec& ref) = 0;

      /**
       * \brief initialise extra dynamic interface
       */
      virtual void init(InitStage stage) = 0;

      /**
       * \brief initialise order one parameters
       */
      virtual void initz() = 0;

  };

}

#endif /* _EXTRADYNAMIC_INTERFACE_H_ */


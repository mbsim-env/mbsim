/* Copyright (C) 2004-2011 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef REVCARDAN_H_
#define REVCARDAN_H_

#include "mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1s_33_rcm/angles.h"

namespace MBSimFlexibleBody {

  /*! 
   * \brief reversed kardan parametrisation
   * \author Thorsten Schindler
   * \date 2009-04-24 initial commit (Thorsten Schindler)
   */
  class RevCardan : public Angles {	
    public:
      /*! 
       * \brief constructor
       */
      RevCardan();

      /*!
       * \brief destructor
       */
      virtual ~RevCardan();

      /* INTERFACE OF ROTATION */
      virtual int getqSize() const { return 3; }
      /********************************************************/

      /* INTERFACE OF ANGLES */
      virtual fmatvec::Vec computet(const fmatvec::Vec& q) const;		
      virtual fmatvec::Vec computen(const fmatvec::Vec& q) const;		
      virtual fmatvec::Vec computeb(const fmatvec::Vec& q) const;
      virtual fmatvec::Vec computentil(const fmatvec::Vec& q) const;		
      virtual fmatvec::Vec computebtil(const fmatvec::Vec& q) const;	
      virtual fmatvec::SqrMat computetq(const fmatvec::Vec& q) const;		
      virtual fmatvec::SqrMat computenq(const fmatvec::Vec& q) const;		
      virtual fmatvec::SqrMat computebq(const fmatvec::Vec& q) const;
      virtual fmatvec::SqrMat computentilq(const fmatvec::Vec& q) const;		
      virtual fmatvec::SqrMat computebtilq(const fmatvec::Vec& q) const;
      virtual fmatvec::Mat computetq2(const fmatvec::Vec& q) const;		
      virtual fmatvec::Mat computenq2(const fmatvec::Vec& q) const;		
      virtual fmatvec::Mat computebq2(const fmatvec::Vec& q) const;
      virtual fmatvec::Mat computentilq2(const fmatvec::Vec& q) const;		
      virtual fmatvec::Mat computebtilq2(const fmatvec::Vec& q) const;
      /********************************************************/			
  };

}

#endif /*REVCARDAN_H_*/


/* Copyright (C) 2004-2006 Mathias Bachmayer
 
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
 * Contact:
 *    mfoerg@users.berlios.de
 *
 */

#ifndef _DIB_H_
#define _DIB_H_

#include "fmatvec.h"
using namespace fmatvec;

namespace MBSim {

  /*! \brief Class for describing external signals */
  class DataInterfaceBase {
    protected:
      /** merge multidimensional signals */
      double SigSize;
      /** name */
      string name;
      
    public:
      /*! Constructor */
      DataInterfaceBase() {}
      /*! Destructor */
      virtual ~DataInterfaceBase() {}
      /*! Get name of DataInterface */
      const string& getName() {return name;}
      /*! Set name of DataInterface */
      void setName(const string& name_) {name=name_;}
      /*! Calculate multidimensional signal with respect to one variable */
      virtual Vec operator()(double t) = 0;
      /*! Get signal size */
      double  getSigSize(){return SigSize;}
      /*! Update signal */
      virtual void update(double t) {}
  };

}

#endif /* _DIB_H_ */

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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _POINTER_H_
#define _POINTER_H_

#include <boost/smart_ptr/shared_ptr.hpp>

namespace MBSimFlexibleBody {

class RevCardan;
class Trafo33RCM;
class Weight33RCM;

typedef boost::shared_ptr<RevCardan> RevCardanPtr;
typedef boost::shared_ptr<Trafo33RCM> Trafo33RCMPtr;
typedef boost::shared_ptr<Weight33RCM> Weight33RCMPtr;

}

#endif /* _POINTER_H_ */

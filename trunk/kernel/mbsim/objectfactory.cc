/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: friedrich.at.gc@googlemail.com
 */

#include <config.h>
#include <mbsim/objectfactory.h>

using namespace fmatvec;

namespace MBSim {

ObjectFactory& ObjectFactory::instance() {
  static ObjectFactory of;
  return of;
}

void ObjectFactory::registerXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc, DeallocateFkt dealloc) {
  // check if name was already registred with the same &allocate<CreateType>: if yes return and do not add it twice
  for(VectorIt it=instance().registeredType.begin(); it!=instance().registeredType.end(); it++) {
    // skip type with wrong key value get<0>()
    if(it->get<0>()!=name) continue;

    if(it->get<1>()==alloc)
      return;
  }
  // name is not registred with &allocate<CreateType>: register it
  instance().registeredType.push_back(VectorContent(name, alloc, dealloc));
}

void ObjectFactory::deregisterXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc) {
  // dereg the element which as a name of 'name' AND a alloc function of 'alloc'
  for(VectorIt it=instance().registeredType.begin(); it!=instance().registeredType.end(); it++)
    if(it->get<0>()==name && it->get<1>()==alloc) {
      instance().registeredType.erase(it);
      return;
    }
}

}

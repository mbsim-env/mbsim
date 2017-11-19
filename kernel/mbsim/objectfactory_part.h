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

#ifndef _MBSIM_OBJECTFACTORY_PART_H_
#define _MBSIM_OBJECTFACTORY_PART_H_

namespace fmatvec {
  class Atom;
}

namespace MBXMLUtils {
  class FQN;
}

namespace MBSim {

//! Base wrapper class to allocate an object derived from fmatvec::Atom.
//! We can use just a function pointer, fmatvec::Atom* (*allocateFct)(), for the same purpose.
//! But for other more complex allocator functions (e.g. Python objects using swig directors)
//! this is not flexible enought since we need to proved a custom operator==.
struct AllocateBase {
  virtual ~AllocateBase() = default;
  //! Implement this function to allocate a new object
  virtual fmatvec::Atom* operator()() const = 0;
  //! Implement this function to compare whether this class and the instance other allocate
  //! the same object type when calling operator().
  virtual bool operator==(const AllocateBase& other) const = 0;
};

//! Base wrapper class to deallocate an object derived from fmatvec::Atom.
//! See also AllocateBase.
struct DeallocateBase {
  virtual ~DeallocateBase() = default;
  //! Implement this function to deallocate the object obj
  virtual void operator()(fmatvec::Atom *obj) const = 0;
};

void registerClass_internal(const MBXMLUtils::FQN &name, const AllocateBase *alloc, const DeallocateBase *dealloc);

void deregisterClass_internal(const MBXMLUtils::FQN &name, const AllocateBase *alloc);



template<class EV> void registerEnum_internal(const MBXMLUtils::FQN &name, const EV& value);

template<class EV> void deregisterEnum_internal(const MBXMLUtils::FQN &name);

}

#endif

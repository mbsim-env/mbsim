/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _MBSIM_PTR_H_
#define _MBSIM_PTR_H_

#include <mbsim/functions/function.h>
#include <mbsim/functions/basic_functions.h>

namespace MBSim {

/*! A "smart" pointer to a object of type T.
 * An Ptr<T> object behaves like a normal pointer: use the dereferenz operator * or ->.
 * Additionally a "convert pointer assignment" operator can be spezialized to cast from type S to T (see below).
 */
template<typename T>
class Ptr {
   public:
      //! @name member functions operating on type T
      ///@{
      //! default ctor
      Ptr() : p(NULL) {}
      //! copy ctor
      Ptr(const Ptr<T> &src) : p(src.p) {}
      //! pointer ctor
      Ptr(T* src) : p(src) {}
      //! assignment
      Ptr<T>& operator=(const Ptr<T> &src) { p=src.p; return *this; }
      //! pointer assignment
      Ptr<T>& operator=(T *src) { p=src; return *this; }
      //! dereference
      T& operator*() const { return *p; }
      //! structure dereference
      T* operator->() const { return p; }
      //! get pointer (convinience)
      T* get() const { return p; }
      //! bool cast
      operator bool() const { return p!=NULL; }
      ///@}


      //! @name convert member functions operating on arbitary type S (casting from type S to T)
      ///@{
      //! convert assignment (calls the convert pointer assignment)
      template<typename S>
      Ptr<T>& operator=(const Ptr<S> &src) { return operator=(src.get()); }
      //! convert copy ctor (calls the convert pointer assignment)
      template<typename S>
      Ptr(const Ptr<S> &src) { operator=(src.get()); }
      //! convert pointer ctor (calls the convert pointer assignment)
      template<typename S>
      Ptr(S* src) { operator=(src); }

      /*! convert pointer assignment.
       * This function must be explicitly spezialized for the type pair T, S for each
       * combination of a cast from type S to type T. */
      template<typename S>
      Ptr<T>& operator=(S *src);
      ///@}

   private:
      T *p;
};

/*! explizit spezialisation of the convert pointer assignment from
 * S=Function<double(double)> to T=Function<fmatvec::VecV(double)> using
 * the helper class VectorValuedFunction.
 */
template<>
template<>
Ptr<Function<fmatvec::VecV(double)> >& Ptr<Function<fmatvec::VecV(double)> >::operator=(Function<double(double)> *src) {
   VectorValuedFunction<fmatvec::VecV> *vvf=new VectorValuedFunction<fmatvec::VecV>();
   vvf->addComponent(src);
   p=vvf;
   return *this;
}

// other "pointer casts" e.g. from Function<double(VecV)> to Function<double(double)> may be added here
// like the one above.

}

#endif

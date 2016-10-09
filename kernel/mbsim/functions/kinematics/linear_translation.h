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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _LINEAR_TRANSLATION_H_
#define _LINEAR_TRANSLATION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<class Arg>
  class LinearTranslation : public Function<fmatvec::Vec3(Arg)> {
    using B = fmatvec::Function<fmatvec::Vec3(Arg)>; 
    private:
      typename B::DRetDArg A;
      fmatvec::Vec3 b;
      fmatvec::Vec3 zeros(const typename B::DRetDArg &x) { return fmatvec::Vec3(x.rows()); }
    public:
      LinearTranslation() { }
      LinearTranslation(const typename B::DRetDArg &A_) : A(A_) { }
      LinearTranslation(const typename B::DRetDArg &A_, const fmatvec::Vec3 &b_) : A(A_), b(b_) { }
      int getArgSize() const { return A.cols(); }
      fmatvec::Vec3 operator()(const Arg &arg) { return A*arg+b; }
      typename B::DRetDArg parDer(const Arg &arg) { return A; }
      typename B::DRetDArg parDerDirDer(const Arg &arg1Dir, const Arg &arg1) { return typename B::DRetDArg(A.rows(),A.cols()); }
      typename B::DDRetDDArg parDerParDer(const double &arg) { THROW_MBSIMERROR("parDerParDer is not available for given template parameters."); }
      bool constParDer() const { return true; }
      void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"translationVectors");
        A=FromMatStr<typename B::DRetDArg>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str());
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"offset");
        b=e?FromMatStr<fmatvec::Vec3>::cast((MBXMLUtils::X()%MBXMLUtils::E(e)->getFirstTextChild()->getData()).c_str()):zeros(A);
      }
      void setSlope(const typename B::DRetDArg &A_) { A = A_; }
      void setIntercept(const fmatvec::Vec3 &b_) { b = b_; }
  };

  template<>
  inline fmatvec::Vec3 LinearTranslation<double>::parDerDirDer(const double &arg1Dir, const double &arg1) { return fmatvec::Vec3(); }
  template<>
  inline fmatvec::Vec3 LinearTranslation<double>::parDerParDer(const double &arg) { return fmatvec::Vec3(); }

}

#endif

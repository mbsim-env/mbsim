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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _FOURIER_FUNCTION_H_
#define _FOURIER_FUNCTION_H_

#include "mbsim/functions/function.h"
#include "mbsim/utils/utils.h"

namespace MBSim {

  template<typename Sig> class FourierFunction; 

  template<typename Ret, typename Arg>
  class FourierFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = fmatvec::Function<Ret(Arg)>; 

    public:
      FourierFunction() = default;
      FourierFunction(double f_, const fmatvec::VecV &a_, const fmatvec::VecV &b_, double a0_=0, bool amplitudePhaseAngleForm_=false) : f(f_), a0(a0_), a(a_), b(b_), amplitudePhaseAngleForm(amplitudePhaseAngleForm_) { }
      int getArgSize() const override { return 1; }
      std::pair<int, int> getRetSize() const override { return std::make_pair(1,1); }
      Ret operator()(const Arg& t_) override {
        double t = ToDouble<Arg>::cast(t_);
        double y = a0/2;
        double Om = 2.*M_PI*f;
        for(int i=0; i<a.size(); i++) {
          int k = i+1;
          double phi = k*Om*t;
          y += a(i)*cos(phi)+b(i)*sin(phi);
        }
        return FromDouble<Ret>::cast(y);
      }
      typename B::DRetDArg parDer(const Arg &t_) override {  
        double t = ToDouble<Arg>::cast(t_);
        double yd = 0;
        double Om = 2.*M_PI*f;
        for(int i=0; i<a.size(); i++) {
          int k = i+1;
          double phi = k*Om*t;
          yd += k*Om*(b(i)*cos(phi)-a(i)*sin(phi));
        }
        return FromDouble<typename B::DRetDArg>::cast(yd);
      }
      void initializeUsingXML(xercesc::DOMElement * element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"frequency");
        f=MBXMLUtils::E(e)->getText<double>();
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a0");
        if(e) a0=MBXMLUtils::E(e)->getText<double>();
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"a");
        if(e) {
          a = MBXMLUtils::E(e)->getText<fmatvec::Vec>();
          e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"b");
          b = MBXMLUtils::E(e)->getText<fmatvec::Vec>();
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"ab");
        if(e) {
          fmatvec::MatV xy = MBXMLUtils::E(e)->getText<fmatvec::Mat>();
          assert(xy.cols() == 2);
          a = xy.col(0);
          b = xy.col(1);
        }
        e = MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"amplitudePhaseAngleForm");
        if(e) amplitudePhaseAngleForm = MBXMLUtils::E(e)->getText<bool>();
      }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Arg)>::init(stage, config);
        if(stage == Element::preInit) {
          if(amplitudePhaseAngleForm) {
            for(int i=0; i<a.size(); i++) {
              double buf = a.e(i);
              a(i) = buf*sin(b(i));
              b(i) = buf*cos(b(i));
            }
          }
        }
      }
    protected:
      double f;
      double a0;
      fmatvec::VecV a, b;
      bool amplitudePhaseAngleForm;
      int size;
    private:
  };

}

#endif

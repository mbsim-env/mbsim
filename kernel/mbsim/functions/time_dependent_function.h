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

#ifndef TIME_DEPENDENT_FUNCTION_H_
#define TIME_DEPENDENT_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template <class Ret> 
  class TimeDependentFunction : public Function<Ret(fmatvec::VecV,double)> {
    using B = fmatvec::Function<Ret(fmatvec::VecV,double)>; 
    private:
      Function<Ret(double)> *f;
      int n{0};
    public:
      TimeDependentFunction(Function<Ret(double)> *f_=NULL) : f(f_) {
        if(f)
          f->setParent(this);
      }
      ~TimeDependentFunction() override { delete f; }
      int getArg1Size() const override { return 0;}
      int getArg2Size() const override { return 1; }
      Ret operator()(const fmatvec::VecV &arg1, const double &arg2) override {return (*f)(arg2); }
      typename B::DRetDArg1 parDer1(const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg1(n,getArg1Size()); }
      typename B::DRetDArg2 parDer2(const fmatvec::VecV &arg1, const double &arg2) override {return f->parDer(arg2); }
      typename B::DDRetDDArg2 parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) override { return f->parDerParDer(arg2); }
      typename B::DRetDArg2 parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg2(n); }
      typename B::DDRetDArg1DArg2 parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DDRetDArg1DArg2(n,getArg1Size()); }
      typename B::DRetDArg1 parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg1(n,getArg1Size()); }
      bool constParDer1() const override { return true; }
      bool constParDer2() const override { return f->constParDer(); }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(fmatvec::VecV,double)>::init(stage, config);
        f->init(stage, config);
        if(stage == Element::preInit)
          n = (*f)(0).rows();
      }
  };

}

#endif

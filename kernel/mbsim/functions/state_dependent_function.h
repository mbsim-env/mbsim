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

#ifndef STATE_DEPENDENT_FUNCTION_H_
#define STATE_DEPENDENT_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template <class Ret>
  class StateDependentFunction : public Function<Ret(fmatvec::VecV,double)> {
    using B = fmatvec::Function<Ret(fmatvec::VecV,double)>; 
    private:
      Function<Ret(fmatvec::VecV)> *f;
      int n{0};
    public:
      StateDependentFunction(Function<Ret(fmatvec::VecV)> *f_=NULL) : f(f_) { 
        if(f)
          f->setParent(this);
      }
      ~StateDependentFunction() override { delete f; }
      int getArg1Size() const override { return f->getArgSize();}
      int getArg2Size() const override { return 0; }
      std::pair<int, int> getRetSize() const override { return f->getRetSize(); }
      Ret operator()(const fmatvec::VecV &arg1, const double &arg2) override {return (*f)(arg1); }
      typename B::DRetDArg1 parDer1(const fmatvec::VecV &arg1, const double &arg2) override { return f->parDer(arg1); }
      typename B::DRetDArg2 parDer2(const fmatvec::VecV &arg1, const double &arg2) override {return typename B::DRetDArg2(n); }
      typename B::DRetDArg2 parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg2(n); }
      typename B::DRetDArg2 parDer2DirDer2(const double &arg2Dir, const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg2(n); }
      typename B::DRetDArg1 parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) override { return f->parDerDirDer(arg1Dir,arg1); }
      typename B::DRetDArg1 parDer1DirDer2(const double &arg2Dir, const fmatvec::VecV &arg1, const double &arg2) override { return typename B::DRetDArg1(n,getArg1Size()); }
      bool constParDer1() const override { return f->constParDer(); }
      bool constParDer2() const override { return true; }
      Function<Ret(fmatvec::VecV)>* getFunction() const { return f; }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(fmatvec::VecV,double)>::init(stage, config);
        f->init(stage, config);
        if(stage == Element::preInit)
          n = f->getRetSize().first;
      }
      void setDynamicSystemSolver(DynamicSystemSolver* sys) override {
        Function<Ret(fmatvec::VecV,double)>::setDynamicSystemSolver(sys);
	f->setDynamicSystemSolver(sys);
      }
  };

}

#endif

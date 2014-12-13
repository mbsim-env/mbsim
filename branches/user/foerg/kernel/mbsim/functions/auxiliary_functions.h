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

#ifndef AUXILIARY_FUNCTIONS_H_
#define AUXILIARY_FUNCTIONS_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template <class Ret>
    class StateDependentFunction : public Function<Ret(fmatvec::VecV,double)> {
      private:
        Function<Ret(fmatvec::VecV)> *f;
        int n;
      public:
        StateDependentFunction(Function<Ret(fmatvec::VecV)> *f_=NULL) : f(f_), n(0) { 
          if(f)
            f->setParent(this);
        }
        ~StateDependentFunction() { delete f; }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return f->getArgSize();}
        typename fmatvec::Size<double>::type getArg2Size() const { return 0; }
        Ret operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg1); }
        typename fmatvec::Der<Ret, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return f->parDer(arg1); }
        typename fmatvec::Der<Ret, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return typename fmatvec::Der<Ret, double>::type(n); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type(n); }
        typename fmatvec::Der<Ret, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<Ret, double>::type(n); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<typename fmatvec::Der<Ret, fmatvec::VecV>::type, double>::type(n,getArg1Size()); }
        typename fmatvec::Der<Ret, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return f->parDerDirDer(arg1Dir,arg1); }
        bool constParDer1() const { return f->constParDer(); }
        bool constParDer2() const { return true; }
        Function<Ret(fmatvec::VecV)>* getFunction() const { return f; }
        void init(Element::InitStage stage) {
          Function<Ret(fmatvec::VecV,double)>::init(stage);
          f->init(stage);
          if(stage == Element::preInit)
            n = (*f)(fmatvec::VecV(getArg1Size())).rows();
        }
        std::vector<Element*> getDependencies() const { return f->getDependencies(); }
    };

   template <class Ret>
    class TimeDependentFunction : public Function<Ret(fmatvec::VecV,double)> {
      private:
        Function<Ret(double)> *f;
        int n;
      public:
        TimeDependentFunction(Function<Ret(double)> *f_=NULL) : f(f_), n(0) {
          if(f)
            f->setParent(this);
        }
        ~TimeDependentFunction() { delete f; }
        typename fmatvec::Size<fmatvec::VecV>::type getArg1Size() const { return 0;}
        typename fmatvec::Size<double>::type getArg2Size() const { return 1; }
        Ret operator()(const fmatvec::VecV &arg1, const double &arg2) {return (*f)(arg2); }
        typename fmatvec::Der<Ret, fmatvec::VecV>::type parDer1(const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<Ret, fmatvec::VecV>::type(n,getArg1Size()); }
        typename fmatvec::Der<Ret, double>::type parDer2(const fmatvec::VecV &arg1, const double &arg2) {return f->parDer(arg2); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, double>::type, double>::type parDer2ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return f->parDerParDer(arg2); }
        typename fmatvec::Der<Ret, double>::type parDer2DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<Ret, double>::type(n); }
        typename fmatvec::Der<typename fmatvec::Der<Ret, fmatvec::VecV>::type, double>::type parDer1ParDer2(const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<typename fmatvec::Der<Ret, fmatvec::VecV>::type, double>::type(n,getArg1Size()); }
        typename fmatvec::Der<Ret, fmatvec::VecV>::type parDer1DirDer1(const fmatvec::VecV &arg1Dir, const fmatvec::VecV &arg1, const double &arg2) { return typename fmatvec::Der<Ret, fmatvec::VecV>::type(n,getArg1Size()); }
        bool constParDer1() const { return true; }
        bool constParDer2() const { return f->constParDer(); }
        void init(Element::InitStage stage) {
          Function<Ret(fmatvec::VecV,double)>::init(stage);
          f->init(stage);
          if(stage == Element::preInit)
            n = (*f)(0).rows();
        }
        std::vector<Element*> getDependencies() const { return f->getDependencies(); }
    };

}

#endif


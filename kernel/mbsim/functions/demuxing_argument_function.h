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

#ifndef _NESTED_FUNCTION_H_
#define _NESTED_FUNCTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  template<typename Sig> class DemuxingArgumentFunction; 

  /**
   * Defines a function \f$ F(X) = f_1(x_i) \f$ or \f$ F(X) = f_2(x_i,x_j) \f$
   * with the argument vector \f$ X = ( ..., x_i, ... , x_j, ... )^T being forwarded
   * to the inner function \f$ f_1 \f$ taking 1 argument or 
   *                       \f$ f_2 \f$ taking 2 arguments. The indices \f$ i \f$ and \f$ j \$f must be user defined.
   */
  template<typename Ret, typename Arg> 
  class DemuxingArgumentFunction<Ret(Arg)> : public Function<Ret(Arg)> {
    using B = Function<Ret(Arg)>; 
    public:
      DemuxingArgumentFunction() = default;
      DemuxingArgumentFunction(Function<Ret(double)> *f1_, Function<Ret(double,double)> *f2_) {
        set1ArgFunction(f1_);
        set2ArgFunction(f2_);
      }
      ~DemuxingArgumentFunction() override {
        delete fo1;
        delete fo2;
      }
      int getArgSize() const override { return 1; }
      std::pair<int, int> getRetSize() const override {
        return fo1?fo1->getRetSize():fo2->getRetSize();
      }
      Ret operator()(const Arg &arg) override {
        const double& a1 = arg(argIdx1);
        if( fo1 )
          return (*fo1)(a1);
        const double& a2 = arg(argIdx2);
        return (*fo2)(a2,a2);
      }
      typename B::DRetDArg parDer(const Arg &arg) override {
        if( fo1 )
          return fo1->parDer(arg(argIdx1));
        return fo2->parDer1(arg(argIdx1),arg(argIdx2)) + fo2->parDer2(arg(argIdx1),arg(argIdx2));
      }
      typename B::DRetDArg parDerDirDer(const Arg &argDir, const Arg &arg) override {
        if( fo1 )
          return fo1->parDerDirDer(argDir(argIdx1), arg(argIdx1));
        return fo2->parDer1DirDer1(argDir(argIdx1), arg(argIdx1),arg(argIdx2)) * argDir(argIdx1) +
               fo2->parDer1DirDer2(argDir(argIdx2), arg(argIdx1),arg(argIdx2)) * argDir(argIdx1) +
               fo2->parDer2DirDer1(argDir(argIdx1), arg(argIdx1),arg(argIdx2)) * argDir(argIdx2) +
               fo2->parDer2DirDer2(argDir(argIdx2), arg(argIdx1),arg(argIdx2)) * argDir(argIdx2);
      }
      void setArgumentIndex1(size_t argIdx1_) { argIdx1 = argIdx1_; }
      void setArgumentIndex2(size_t argIdx2_) { argIdx2 = argIdx2_; }
      void set1ArgFunction(Function<Ret(double)> *fo_) {
        fo1 = fo_;
        fo1->setParent(this);
        fo1->setName("OneArgumentFunction");
      }
      void set2ArgFunction(Function<Ret(double,double)> *fo_) {
        fo2 = fo_;
        fo2->setParent(this);
        fo2->setName("TwoArgumentFunction");
      }
      void initializeUsingXML(xercesc::DOMElement *element) override {
        xercesc::DOMElement *e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"oneArgumentFunction");
        if(e)
          set1ArgFunction(ObjectFactory::createAndInit<Function<Ret(double)>>(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"twoArgumentFunction");
        if(e)
          set2ArgFunction(ObjectFactory::createAndInit<Function<Ret(double,double)>>(e->getFirstElementChild()));
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"argumentIndex1");
        if(e)
          setArgumentIndex1(MBXMLUtils::E(e)->getText<size_t>());
        e=MBXMLUtils::E(element)->getFirstElementChildNamed(MBSIM%"argumentIndex2");
        if(e)
          setArgumentIndex2(MBXMLUtils::E(e)->getText<size_t>());
      }
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<Ret(Arg)>::init(stage, config);
        if(fo1) fo1->init(stage, config);
        if(fo2) fo2->init(stage, config);
      }
    private:
      Function<Ret(double)>        *fo1{nullptr};
      Function<Ret(double,double)> *fo2{nullptr};
      size_t argIdx1;
      size_t argIdx2;
  };
}

#endif

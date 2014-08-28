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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef FMISCALARVARIABLEUNIT_INL_
#define FMISCALARVARIABLEUNIT_INL_
#include "fmi_scalar_variable_unit.h"
#include <fmatvec/fmatvec.h>
#include <mbsim/extern_generalized_io.h>
#include <mbsimControl/extern_signal_source.h>
#include <mbsimControl/extern_signal_sink.h>

namespace fmi {
  /************************************************************
  *   Specialization for double/integer/string/bool
  ************************************************************/
  template<class T>
  class ScalarVariableUnit<T, T> : public InterfaceScalarVariable<T> {
  public:
    ScalarVariableUnit(T obj_):obj(obj_){};
    virtual ~ ScalarVariableUnit(){};

    void get(T& value, int i=0){value = obj;};
    void set(T value, int i=0){obj = value;};
    int size(){return 1;};
    std::vector<Variability> getVariability(){std::vector<Variability> out(1); out[0]=fmiParameter; return out;};
    std::vector<Causality> getCausality(){std::vector<Causality> out(1); out[0]=fmiNone; return out;};

  private:
    T obj;
  };
  /************************************************************
  *   Specialization for ExternGeneralizedIO
  ************************************************************/
  template<> class ScalarVariableUnit<double, MBSim::ExternGeneralizedIO*> : public InterfaceScalarVariable<double> {
    public:
      ScalarVariableUnit(MBSim::ExternGeneralizedIO* obj_):obj(obj_){};
      virtual ~ ScalarVariableUnit(){};

      void get(double& value, int i=0){
        if(i>=3) return;
        switch(i) {
          case 0 : return; break;
          case 1 : value=obj->getGeneralizedPosition(); break;
          case 2 : value=obj->getGeneralizedVelocity(); break;
        }
      }
      void set(double value, int i=0){
        if(i>=3) return;
        switch(i) {
          case 0 : obj->setGeneralizedForce(value);break;
          case 1 : break;
          case 2 : break;
        }
      }
      int size(){return 3;};
      std::vector<Variability> getVariability() {
        std::vector<Variability> out(3);
        out.assign(3,fmiContinuous);
        return out;
      };
      std::vector<Causality> getCausality(){
        std::vector<Causality> out(3);
        out[0]=fmiInput;
        out[1]=fmiOutput;
        out[2]=fmiOutput;
        return out;
      }
    private:
      MBSim::ExternGeneralizedIO* obj;
  };
  /************************************************************
  *   Specialization for ExternSignalSource
  ************************************************************/
  template<> class ScalarVariableUnit<double, MBSimControl::ExternSignalSource*> : public InterfaceScalarVariable<double> {
    public:
      ScalarVariableUnit(MBSimControl::ExternSignalSource* obj_):obj(obj_){fmatvec::Vec tmp=obj->getSignal();sizeNumber=tmp.size();};
      virtual ~ ScalarVariableUnit(){};

      void get(double& value, int i=0){
        fmatvec::Vec tmp=obj->getSignal();
        if(i>=sizeNumber) return;
        value=tmp(i);
      }
      void set(double value, int i=0){
        fmatvec::Vec tmp=obj->getSignal();
        if(i>=sizeNumber) return;
        tmp(i)=value;
        obj->setSignal(tmp);
      }
      int size(){return sizeNumber;};
      std::vector<Variability> getVariability() {
        std::vector<Variability> out(sizeNumber,fmiContinuous);
        return out;
      };
      std::vector<Causality> getCausality(){
        std::vector<Causality> out(sizeNumber,fmiInput);
        return out;
      }
    private:
      MBSimControl::ExternSignalSource* obj;
      int sizeNumber;
  };
  /************************************************************
  *   Specialization for ExternSignalSink
  ************************************************************/
  template<> class ScalarVariableUnit<double, MBSimControl::ExternSignalSink*> : public InterfaceScalarVariable<double> {
    public:
      ScalarVariableUnit(MBSimControl::ExternSignalSink* obj_):obj(obj_){fmatvec::Vec tmp=obj->getSignal();sizeNumber=tmp.size();};
      virtual ~ ScalarVariableUnit(){};

      void get(double& value, int i=0){
        fmatvec::Vec tmp=obj->getSignal();
        if(i>=sizeNumber) return;
        value=tmp(i);
      }
      void set(double value, int i=0){
      }
      int size(){return sizeNumber;};
      std::vector<Variability> getVariability() {
        std::vector<Variability> out(sizeNumber,fmiContinuous);
        return out;
      };
      std::vector<Causality> getCausality(){
        std::vector<Causality> out(sizeNumber,fmiOutput);
        return out;
      }
    private:
      MBSimControl::ExternSignalSink* obj;
      int sizeNumber;
  };


} /* namespace fmi */

#endif

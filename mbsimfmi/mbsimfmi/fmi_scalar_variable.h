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

#ifndef FMISCALARVARIABLE_H_
#define FMISCALARVARIABLE_H_

#include "fmiModelTypes.h"
#include "fmi_scalar_variable_container.h"
#include <cfloat>

namespace fmi {

/**
 * \brief Base class for scalar variable vectors in FMI interface.
 *        Contains properties data of a scalar value (Causality, Variability, hasStartValue)
 * \author F.Péan
 * \date 2014-03-28 Adding doxygen comments (F.Péan)
 */
class BaseScalarVariable {
public:
  BaseScalarVariable(){}
  virtual ~BaseScalarVariable(){}

  /* INTERFACE */
  const fmiBoolean isVariability(fmiInteger i, Variability v) const { return (variability[i]==v)?fmiTrue:fmiFalse; }
  const fmiBoolean isConstant(fmiInteger i) const { return (variability[i]==fmiConstant)?fmiTrue:fmiFalse; }
  const fmiBoolean isParameter(fmiInteger i) const { return (variability[i]==fmiParameter)?fmiTrue:fmiFalse; }
  const fmiBoolean isDiscrete(fmiInteger i) const { return (variability[i]==fmiDiscrete)?fmiTrue:fmiFalse; }
  const fmiBoolean isContinuous(fmiInteger i) const { return (variability[i]==fmiContinuous)?fmiTrue:fmiFalse; }

  const fmiBoolean isCausality(fmiInteger i, Causality c) const { return (causality[i]==c)?fmiTrue:fmiFalse; }
  const fmiBoolean isInput(fmiInteger i) const { return (causality[i]==fmiInput)?fmiTrue:fmiFalse; }
  const fmiBoolean isOutput(fmiInteger i) const { return (causality[i]==fmiOutput)?fmiTrue:fmiFalse; }
  const fmiBoolean isInternal(fmiInteger i) const { return (causality[i]==fmiInternal)?fmiTrue:fmiFalse; }
  const fmiBoolean isNone(fmiInteger i) const { return (causality[i]==fmiNone)?fmiTrue:fmiFalse; }

  void setVariability(fmiInteger idx, Variability variability_) {variability.at(idx)=variability_;}
  void setCausality(fmiInteger idx, Causality causality_) {causality.at(idx)=causality_;}
  void setStartBool(fmiInteger idx, fmiBoolean b) { hasStartValue.at(idx)=b; }

  void setVariability(Variability variability_) {variability.push_back(variability_);}
  void setCausality(Causality causality_) {causality.push_back(causality_);}

  int inputSize() { int count=0;for(uint i =0;i<causality.size();i++) isInput(i)?count++:count; return count;};
  int outputSize(){ int count=0;for(uint i =0;i<causality.size();i++) isOutput(i)?count++:count; return count;};
  int internalSize() { int count=0;for(uint i =0;i<causality.size();i++) isInternal(i)?count++:count; return count;};
  int noneSize()  { int count=0;for(uint i =0;i<causality.size();i++) isNone(i)?count++:count; return count;};

  const fmiBoolean hasStartBool(fmiInteger i) { return hasStartValue[i]?fmiTrue:fmiFalse; }
  void setStartBool(fmiBoolean b) {hasStartValue.push_back(b); }

  virtual void resize(fmiInteger size) {variability.resize(size);causality.resize(size);hasStartValue.resize(size);}

  virtual void remove(fmiInteger idx) {};
  virtual void remove(void) {};

protected:
  void rmVariability(fmiInteger idx)  { variability.erase(variability.begin()+idx);}
  void rmCausality(fmiInteger idx)    { causality.erase(causality.begin()+idx);}
  void rmStartBool(fmiInteger idx)    { hasStartValue.erase(hasStartValue.begin()+idx); }

  void rmVariability(void)  { variability.pop_back(); }
  void rmCausality(void)    { causality.pop_back(); }
  void rmStartBool(void)    { hasStartValue.pop_back(); }
  /*****************************************************/
  std::vector<Variability> variability;
  std::vector<Causality> causality;
  std::vector<fmiBoolean> hasStartValue;
};

/**
 * \brief Template class for scalar variable vectors in FMI interface.
 *        Extends base to type dependent data
 * \author F.Péan
 * \date 2014-03-28 Adding doxygen comments (F.Péan)
 */
template <typename T>
class ScalarVariable : public BaseScalarVariable, public InterfaceScalarVariable<T> {
  typedef T Tin;
  typedef std::vector<T> Vec;
  typedef BaseScalarVariable Inherit;

public:
  ScalarVariable():BaseScalarVariable(),nullRef() {};
  virtual ~ScalarVariable() {};

  Vec& operator() (void) { return data; };
  const Vec& operator() (void) const { return data; };

  T& operator() (fmiInteger i) { return data.at(i); };
  const T& operator() (fmiInteger i) const { return data.at(i); };

  void resize(fmiInteger size) { data.resize(size); startValue.resize(size); Inherit::resize(size); };

  void remove(fmiInteger idx) { data.erase(data.begin()+idx); rmCausality(idx); rmVariability(idx); };
  void remove(void) { data.pop_back(); rmCausality(); rmVariability(); };

  T nullRef;

  /*
   * \brief init the state of data stored for the type T
   */
  void setup(){
    variability=svc.getVariability();
    causality=svc.getCausality();
    data.resize(svc.size());
    svc.pull(data);
  };
  /*
   * \brief copy data to real place in the model
   */
  void push(){svc.push(data);};
  /*
   * \brief copy data from the model into a single vector stored here as data
   */
  void pull(){svc.pull(data);};

  /* INHERITED INTERFACE */
  void get(T& value, int i=0){value=data.at(i);};
  void set(T value, int i=0) {data.at(i)=value;};
  int size(){return data.size();};
  std::vector<Variability> getVariability(){return variability;};
  std::vector<Causality> getCausality() {return causality;};
  template<class U> void add(U obj){svc.add(obj);}

private:
  //all data of type T
  Vec data;
  //start value of he data
  Vec startValue;
  //interface to specific separate unit making up data
  ScalarVariableContainer<T> svc;
};

} //end namespace fmi

#endif // SCALARVARIABLE_H_

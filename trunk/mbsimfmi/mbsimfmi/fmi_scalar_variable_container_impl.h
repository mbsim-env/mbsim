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

#ifndef FMISCALARVARIABLECONTAINER_INL_
#define FMISCALARVARIABLECONTAINER_INL_

#include "fmiModelTypes.h"
#include "fmi_scalar_variable.h"
#include "fmi_scalar_variable_unit.h"
#include "fmi_scalar_variable_container.h"

//forward declaration of MBSim class used for FMI interface
namespace MBSim {
  class Link;
  class ExternGeneralizedIO;
}
namespace MBSimControl {
  class ExternSignalSource;
  class ExternSignalSink;
}

namespace fmi {

template<class T> template<class U>
void ScalarVariableContainer<T>::add(U in) {
    addCommon(in);
}

template<class T> template <class U>
void ScalarVariableContainer<T>::addCommon(U in) {
    svUnits.push_back(new ScalarVariableUnit<T,U>(in));
    int size = svUnits.back()->size();
    mapping.push_back(mapping.back()+size);
}

//template<class T>
//void ScalarVariableContainer<T>::add(T in) {
//    svUnits.push_back(new ScalarVariableUnit<T,T>(new T(in)));
//    mapping.push_back(mapping.back()+1);
//}

template<class T>
void ScalarVariableContainer<T>::rem() {
    svUnits.pop_back();
    mapping.pop_back();
}

template<class T>
void ScalarVariableContainer<T>::pull(std::vector<T>& vOut) {
  if(vOut.size() != this->size()) vOut.resize(this->size());
  for(size_t i=0; i<mapping.size()-1;i++){
    int j=mapping[i];
    while(j<mapping[i+1]) {
      int local_index = j-mapping[i];
      svUnits[i]->get(vOut[j],local_index);
      j++;
    }
  }
}

template<class T>
void ScalarVariableContainer<T>::push(const std::vector<T>& vIn) {
  for(size_t i=0; i<mapping.size()-1;i++){
    int j=mapping[i];
    while(j<mapping[i+1]) {
      int local_index = j-mapping[i];
      svUnits[i]->set(vIn[j],local_index);
      j++;
    }
  }
}

template<class T>
std::vector<Variability> ScalarVariableContainer<T>::getVariability(){
    std::vector<Variability> out;
    std::vector<Variability> in;
    for(size_t i=0; i<svUnits.size();i++){
      in=svUnits[i]->getVariability();
      out.insert(out.end(),in.begin(),in.end());
    }
    return out;
}

template<class T>
std::vector<Causality> ScalarVariableContainer<T>::getCausality(){
    std::vector<Causality> out;
    std::vector<Causality> in;
    for(size_t i=0; i<svUnits.size();i++){
      in=svUnits[i]->getCausality();
      out.insert(out.end(),in.begin(),in.end());
    }
    return out;
}

} //end namespace fmi



#endif /* FMISCALARVARIABLECONTAINER_INL_ */

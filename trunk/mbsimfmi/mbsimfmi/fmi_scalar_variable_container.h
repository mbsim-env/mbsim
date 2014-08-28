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

#ifndef FMISCALARVARIABLECONTAINER_H_
#define FMISCALARVARIABLECONTAINER_H_

#include <vector>
#include <assert.h>
#include "fmiModelTypes.h"
#include "fmi_scalar_variable_unit.h"

namespace fmi {

/**
 * \brief Class aggregating individual chunk of type T
 *        Provides access from local chunk to a global information
 * \author F.Péan
 * \date 2014-07-28 Adding doxygen comments (F.Péan)
 */
template<class T>
class ScalarVariableContainer : public InterfaceScalarVariable<T> {
public:
  ScalarVariableContainer(){mapping.push_back(0);};
  virtual ~ScalarVariableContainer(){};

  template<class U>
  void add(U in);
  template <class U>
  void addCommon(U in);
//  void add(T in);
  void rem();

  /*
   * \brief copy data from the model into a single vector stored here as data
   */
  void pull(std::vector<T>& vOut);
  /*
   * \brief copy data to real place in the model
   */
  void push(const std::vector<T>& vIn);

  /* INHERITED INTERFACE */
  void get(T& value, int i=0){};
  void set(T value, int i=0){};
  int size(){return mapping.back();};
  std::vector<Variability> getVariability();
  std::vector<Causality> getCausality();

private:
  //stores the overall different objects for this type T in vector of interface to real object in the model
  std::vector<InterfaceScalarVariable<T>* > svUnits;
  //indicates quantity of values stored in every unit, to know until when to loop for every unit
  std::vector<int> mapping;
};

template<> template<>
void ScalarVariableContainer<fmiReal>::add<MBSim::Link*>(MBSim::Link* in);
} //end namespace fmi

#include "fmi_scalar_variable_container.inl"

#endif //FMISCALARVARIABLECONTAINER_H

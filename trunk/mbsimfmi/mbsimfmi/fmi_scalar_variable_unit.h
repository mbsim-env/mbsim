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

#ifndef FMISCALARVARIABLEUNIT_H_
#define FMISCALARVARIABLEUNIT_H_

#include "fmi_enum.h"

namespace fmi {
  
  /**
   * \brief Template class providing common information access
   *        about ScalarVariable system
   * \author F.Péan
   * \date 2014-06-05 Adding doxygen comments (F.Péan)
   */
  template<class T>
  class InterfaceScalarVariable {
  public:
    virtual ~InterfaceScalarVariable(){};
  protected:
    InterfaceScalarVariable(){};
  public:
    /**
     * \brief retrieve the value stored at position [i] in [value]
     * \param value get the actual stored value in this parameter
     * \param i index of the value to retrieve
     */
    virtual void get(T& value, int i=0){};
    /**
     * \brief retrieve the value stored at position [i] in [value]
     * \param value get the actual stored value in this parameter
     * \param i index of the value to retrieve
     */
    virtual void set(T value, int i=0){};
    /**
     * \brief get the size of the data contained
     * \return size of data
     */
    virtual int size(){return 0;};
    /**
     * \brief get the variability (see fmiEnum.h) of the data stored
     * \return vector of the variability
     */
    virtual std::vector<Variability> getVariability(){return std::vector<Variability>();};
    /**
     * \brief get the variability (see fmiEnum.h) of the data stored
     * \return vector of the variability
     */
    virtual std::vector<Causality> getCausality() {return std::vector<Causality>();};
  };

  /**
   * \brief Template class handling access to specific MBSim class it must contains
   *        (e.g ExternGeneralizedIO, ExternSignal...)
   * \author F.Péan
   * \date 2014-06-05 Adding doxygen comments (F.Péan)
   */
  template<class T, class O>
  class ScalarVariableUnit : public InterfaceScalarVariable<T> {
  typedef T TypeIn;
  typedef O ObjectIn;

  public:
    ScalarVariableUnit(ObjectIn obj_):obj(obj_){};
    virtual ~ ScalarVariableUnit(){};

    /* INHERITED INTERFACE */
    void get(T& value, int i=0){};
    void set(T value, int i=0){};
    int size(){return 0;};
    std::vector<Variability> getVariability(){return std::vector<Variability>();};
    std::vector<Causality> getCausality() {return std::vector<Causality>();};
    /*****************************************************/
  private:
    ObjectIn obj;
  };

} /* namespace fmi */

#include "fmi_scalar_variable_unit.inl"

#endif /* FMISCALARVARIABLEUNIT_H_ */

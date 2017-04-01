/* Copyright (C) 2004-2017 MBSim Development Team
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

#ifndef _CELL_ARRAY_H_
#define _CELL_ARRAY_H_

#include "mbsimFlexibleBody/node_based_body.h"
#include "mbsim/functions/time_dependent_function.h"
#include "mbsim/functions/state_dependent_function.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/index.h"
#include "mbsimFlexibleBody/utils/openmbv_utils.h"

namespace MBSimFlexibleBody {

  class ErrorType {
    public:
      ErrorType() {
        throw std::runtime_error("Impossible type.");
      }
  };

  template<typename Dep>
    struct BaseType {
      typedef ErrorType type;
    };

  template<>
    struct BaseType<fmatvec::Vec3> {
      typedef fmatvec::VecV type;
      static int size;
      static fmatvec::Vec3 getEle(xercesc::DOMElement *element) { return MBSim::Element::getVec3(element); }
      static fmatvec::VecV getMat(xercesc::DOMElement *element) { return MBSim::Element::getVec(element); }
    };

  int BaseType<fmatvec::Vec3>::size = 3;

  template<>
    struct BaseType<fmatvec::Vector<fmatvec::Fixed<6>, double> > {
      typedef fmatvec::VecV type;
      static int size;
      static fmatvec::Vector<fmatvec::Fixed<6>, double> getEle(xercesc::DOMElement *element) { return MBSim::Element::getVec(element); }
      static fmatvec::VecV getMat(xercesc::DOMElement *element) { return MBSim::Element::getVec(element); }
    };

  int BaseType<fmatvec::Vector<fmatvec::Fixed<6>, double> >::size = 6;

  template<>
    struct BaseType<fmatvec::SqrMat3> {
      typedef fmatvec::MatVx3 type;
      static int size;
      static fmatvec::SqrMat3 getEle(xercesc::DOMElement *element) { return MBSim::Element::getSqrMat3(element); }
      static fmatvec::MatVx3 getMat(xercesc::DOMElement *element) { return MBSim::Element::getMat(element); }
    };

  int BaseType<fmatvec::SqrMat3>::size = 3;

  template<>
    struct BaseType<fmatvec::SqrMatV> {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::SqrMatV getEle(xercesc::DOMElement *element) { return MBSim::Element::getSqrMat(element); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBSim::Element::getMat(element); }
    };

  int BaseType<fmatvec::SqrMatV>::size = 0;

  template<>
    struct BaseType<fmatvec::Mat3xV> {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::Mat3xV getEle(xercesc::DOMElement *element) { return MBSim::Element::getMat3xV(element); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBSim::Element::getMat(element); }
    };

  int BaseType<fmatvec::Mat3xV>::size = 3;

  template<>
    struct BaseType<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> > {
      typedef fmatvec::MatV type;
      static int size;
      static fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> getEle(xercesc::DOMElement *element) { return MBSim::Element::getMat(element); }
      static fmatvec::MatV getMat(xercesc::DOMElement *element) { return MBSim::Element::getMat(element); }
    };

  int BaseType<fmatvec::Matrix<fmatvec::General, fmatvec::Fixed<6>, fmatvec::Var, double> >::size = 6;

  template <class T>
  class CellArray1D : public std::vector<T> {
    public:
      CellArray1D(int m=0) : std::vector<T>(m) { }
      CellArray1D(const std::vector<T> &array) : std::vector<T>(array) { }
      CellArray1D(const typename BaseType<T>::type &A) { setArray(A); }
//      void setArray(const std::vector<T> &array) { (*this) = array }
      void setArray(const typename BaseType<T>::type &A) {
        int n = A.cols();
        int m = BaseType<T>::size?BaseType<T>::size:n;
        this->resize(A.rows()/m);
        for(unsigned int i=0; i<this->size(); i++)
          (*this)[i] = T(A(fmatvec::RangeV(m*i,m*i+(m-1)),fmatvec::RangeV(0,n-1)));
      }
      virtual void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement* e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"ele") {
          while(e) {
            this->push_back(BaseType<T>::getEle(e));
            e=e->getNextElementSibling();
          }
        }
        else
          setArray(BaseType<T>::getMat(e));
      }
  };
  
  template <class T>
  class CellArray2D : public std::vector<std::vector<T> > {
    public:
      CellArray2D(int m=0) : std::vector<std::vector<T> >(m) { }
      CellArray2D(const std::vector<std::vector<T> > &array) : std::vector<std::vector<T> >(array) { }
      CellArray2D(const typename BaseType<T>::type &A) { setArray(A); }
      void setArray(const typename BaseType<T>::type &A) {
        int n = A.cols();
        int m = BaseType<T>::size?BaseType<T>::size:n;
        int k = 0;
        this->resize(A.rows()/m/n);
        for(unsigned int i=0; i<this->size(); i++) {
          (*this)[i].resize(n);
          for(int j=0; j<n; j++) {
            (*this)[i][j] = T(A(fmatvec::RangeV(m*k,m*k+(m-1)),fmatvec::RangeV(0,n-1)));
            k++;
          }
        }
      }
      virtual void initializeUsingXML(xercesc::DOMElement *element) {
        xercesc::DOMElement *e=element->getFirstElementChild();
        if(MBXMLUtils::E(e)->getTagName()==MBSIMFLEX%"row") {
          while(e) {
            this->push_back(std::vector<T>());
            xercesc::DOMElement *ee=e->getFirstElementChild();
            while(ee) {
              (*this)[this->size()-1].push_back(BaseType<T>::getEle(ee));
              ee=ee->getNextElementSibling();
            }
            e=e->getNextElementSibling();
          }
        }
        else
          setArray(BaseType<T>::getMat(e));
      }
  };

}

#endif

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

#ifndef UTILS_H_
#define UTILS_H_

#include <string>
#include "fmatvec/fmatvec.h"
#include <mbsim/numerics/csparse.h>
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <limits>
#include <vector>
#include <set>

namespace MBSim {

  std::string numtostr(int i);   
  std::string numtostr(double d);   
  template<class Type, class Row, class Col>
    std::string numtostr(fmatvec::Matrix<Type,Row,Col,double> m) {
      std::ostringstream oss;
      oss << "[ ";
      for(int i=0;i<m.rows()-1;i++) {
        for(int j=0;i<m.cols()-1;j++) oss << m.e(i,j) << ", ";
        oss << "\n";
      }
      oss << "]";
      return oss.str(); 
    }

  double degtorad(double alpha);
  double radtodeg(double phi);
  fmatvec::Vec degtorad(fmatvec::Vec alpha);
  fmatvec::Vec radtodeg(fmatvec::Vec phi);
  fmatvec::Vec tildetovec(const fmatvec::SqrMat &A);

  double sign(double x);

  /*!
   * \brief calculates planar angle in [0,2\pi] with respect to Cartesian coordinates of: Arc Tangent (y/x)
   * \param Cartesian x-coordinate
   * \param Cartesian y-coordinate
   * \return angle
   */
  double ArcTan(double x,double y);

  /*!
   * \brief calculate a fmatvec::Mat out of a sparse matrix
   */
  fmatvec::Mat cs2Mat(cs* sparseMat);

  template <class T>
    inline std::string toStr(const T &val) {
      std::stringstream s;
      s << std::setprecision(std::numeric_limits<double>::digits10+1) << val;
      return s.str();
    }

  /*!
   * \brief create a subvector out of a full vector
   *
   * \param origMat original matrix
   * \param indMat  indices that should be used
   * \param diffInd constant index shift
   * \return newMatrix
   *
   * \todo: (much copy work done here! and not implemented for all types / in general)
   */
  fmatvec::VecV subVec(const fmatvec::VecV & origVec, const fmatvec::VecVI & indVec, const int diffInd = 0);

  /*!
   * \brief create a submatrix out of a full matrx (see subVec)
   */
  fmatvec::SqrMatV subMat(const fmatvec::SqrMatV & origMat, const fmatvec::VecVI & indVec, const int diffInd = 0);


  fmatvec::SqrMat subMat2(const fmatvec::SqrMatV & origMat, const fmatvec::VecVI & indVec, const int diffInd = 0);


  inline xercesc::DOMNode* toXML(const std::string &str, xercesc::DOMNode* parent) {
    return parent->getOwnerDocument()->createTextNode(MBXMLUtils::X()%str);
  }

  inline xercesc::DOMNode* toXML(int i, xercesc::DOMNode* parent) {
    return parent->getOwnerDocument()->createTextNode(MBXMLUtils::X()%toStr(i));
  }

  inline xercesc::DOMNode* toXML(unsigned int i, xercesc::DOMNode* parent) {
    return parent->getOwnerDocument()->createTextNode(MBXMLUtils::X()%toStr(i));
  }

  inline xercesc::DOMNode* toXML(double d, xercesc::DOMNode* parent) {
    return parent->getOwnerDocument()->createTextNode(MBXMLUtils::X()%toStr(d));
  }

  template <class T>
    inline xercesc::DOMNode* toXML(const std::vector<T> &x, xercesc::DOMNode* parent) {
      xercesc::DOMElement *ele = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"xmlVector");
      for(unsigned int i=0; i<x.size(); i++) {
        xercesc::DOMElement *elei = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"ele");
        xercesc::DOMText *text = new xercesc::DOMText(toStr(x[i]));
        elei->insertBefore(text, NULL);
        ele->insertBefore(elei, NULL);
      }
      return ele;
    }

  template <class Row>
    inline xercesc::DOMNode* toXML(const fmatvec::Vector<Row,double> &x, xercesc::DOMNode* parent) {
      xercesc::DOMElement *ele = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"xmlVector");
      for(int i=0; i<x.size(); i++) {
        xercesc::DOMDocument *doc=parent->getOwnerDocument();
        xercesc::DOMElement *elei = MBXMLUtils::D(doc)->createElement(MBXMLUtils::PV%"ele");
        xercesc::DOMText *text = doc->createTextNode(MBXMLUtils::X()%toStr(x.e(i)));
        elei->insertBefore(text, NULL);
        ele->insertBefore(elei, NULL);
      }
      return ele;
    }

  template <class Type, class Row, class Col>
    inline xercesc::DOMNode* toXML(const fmatvec::Matrix<Type,Row,Col,double> &A, xercesc::DOMNode* parent) {
      xercesc::DOMElement *ele = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"xmlMatrix");
      for(int i=0; i<A.rows(); i++) {
        xercesc::DOMElement *elei = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"row");
        for(int j=0; j<A.cols(); j++) {
          xercesc::DOMElement *elej = MBXMLUtils::D(parent->getOwnerDocument())->createElement(MBXMLUtils::PV%"ele");
          xercesc::DOMText *text = new xercesc::DOMText(toStr(A.e(i,j)));
          elej->insertBefore(text, NULL);
          elei->insertBefore(elej, NULL);
        }
        ele->insertBefore(elei, NULL);
      }
      return ele;
    }

  template<class T>
    inline std::string funcExt() {
      return "V";
    }

  template < >
    inline std::string funcExt<double>() {
      return "S";
    }

  template <class T>
    void addElementText(xercesc::DOMElement *parent, const MBXMLUtils::FQN &name, const T &value) {
      xercesc::DOMElement *ele = MBXMLUtils::D(parent->getOwnerDocument())->createElement(name);
      ele->insertBefore(toXML(value,parent), NULL);
      parent->insertBefore(ele, NULL);
    }

  template <class Arg>
    class ToDouble {
    };

  template <>
    class ToDouble<double> {
      public:
        static double cast(const double &x) {
          return x;
        }
    };

  template <class Col>
    class ToDouble<fmatvec::Vector<Col,double> > {
      public:
        static double cast(const fmatvec::Vector<Col,double> &x) {
          return x.e(0); 
        }
    };

  template <class Row>
    class ToDouble<fmatvec::RowVector<Row,double> > {
      public:
        static double cast(const fmatvec::RowVector<Row,double> &x) {
          return x.e(0); 
        }
    };

  template <class Ret>
  class FromMatStr {
    public:
      static Ret cast(const char *x) {
        return Ret(x);
      }
  };

  template <>
  class FromMatStr<double> {
    public:
      static double cast(const char *x) {
        return atof(x);
      }
  };

  template <class Ret>
  class FromDouble {
    public:
      static Ret cast(double x) {
        return Ret(1,fmatvec::INIT,x);
      }
  };

  template <>
  class FromDouble<double> {
    public:
      static double cast(double x) {
        return x;
      }
  };

  template <class Ret>
  class FromVecV {
    public:
      static Ret cast(const fmatvec::VecV &x) {
        return x;
      }
  };

  template <>
  class FromVecV<double> {
    public:
      static double cast(const fmatvec::VecV &x) {
        return x(0);
      }
  };

}

#endif

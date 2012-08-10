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
#include "fmatvec.h"
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

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
int min(int i, int j);

/*!
 * \brief calculates planar angle in [0,2\pi] with respect to Cartesian coordinates of: Arc Tangent (y/x)
 * \param Cartesian x-coordinate
 * \param Cartesian y-coordinate
 * \return angle
 */
double ArcTan(double x,double y);

template <class Type, class Row, class Col, class AT>
std::string mat2str(const fmatvec::Matrix<Type,Row,Col,AT> &A) {
  std::stringstream s;
  s << "[";
  for(int i=0; i<A.rows(); i++) {
    for(int j=0; j<A.cols(); j++) {
      s << std::setprecision(12) << A(i,j);
      if(j<A.cols()-1)
        s << ",";
    }
    if(i<A.rows()-1)
      s << ";";
  }
  s << "]";
  return s.str();

}

template <class Row, class AT>
std::string vec2str(const fmatvec::Vector<Row,AT> &x) {
  std::stringstream s;
  s << "[";
  for(int i=0; i<x.size(); i++) {
    s << std::setprecision(12) << x(i);
    if(i<x.size()-1)
      s << ";";
  }
  s << "]";
  return s.str();

}

template <class T>
void addElementText(TiXmlElement *parent, std::string name, T value) {
  std::ostringstream oss;
  oss<<value;
  parent->LinkEndChild(new TiXmlElement(name))->LinkEndChild(new TiXmlText(oss.str()));
}

#endif

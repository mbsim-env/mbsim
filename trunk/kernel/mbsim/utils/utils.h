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

#define PVNS_ "http://openmbv.berlios.de/MBXMLUtils/physicalvariable"
#define PVNS "{"PVNS_"}"

#include <string>
#include "fmatvec.h"
#include "mbxmlutilstinyxml/tinyxml.h"
#include <limits>
#include <vector>
#include <set>

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

template <class T>
inline std::string toStr(const T &val) {
  std::stringstream s;
  s << std::setprecision(std::numeric_limits<double>::digits10+1) << val;
  return s.str();
}

inline MBXMLUtils::TiXmlNode* toXML(const std::string &str) {
  return new MBXMLUtils::TiXmlText(str);
}

inline MBXMLUtils::TiXmlNode* toXML(int i) {
  return new MBXMLUtils::TiXmlText(toStr(i));
}

inline MBXMLUtils::TiXmlNode* toXML(double d) {
  return new MBXMLUtils::TiXmlText(toStr(d));
}

template <class Row>
inline MBXMLUtils::TiXmlNode* toXML(const fmatvec::Vector<Row,double> &x) {
  MBXMLUtils::TiXmlElement *ele = new MBXMLUtils::TiXmlElement(PVNS"xmlVector");
  for(int i=0; i<x.size(); i++) {
    MBXMLUtils::TiXmlElement *elei = new MBXMLUtils::TiXmlElement(PVNS"ele");
    MBXMLUtils::TiXmlText *text = new MBXMLUtils::TiXmlText(toStr(x.e(i)));
    elei->LinkEndChild(text);
    ele->LinkEndChild(elei);
  }
  return ele;
}

template <class Type, class Row, class Col>
inline MBXMLUtils::TiXmlNode* toXML(const fmatvec::Matrix<Type,Row,Col,double> &A) {
  MBXMLUtils::TiXmlElement *ele = new MBXMLUtils::TiXmlElement(PVNS"xmlMatrix");
  for(int i=0; i<A.rows(); i++) {
    MBXMLUtils::TiXmlElement *elei = new MBXMLUtils::TiXmlElement(PVNS"row");
    for(int j=0; j<A.cols(); j++) {
      MBXMLUtils::TiXmlElement *elej = new MBXMLUtils::TiXmlElement(PVNS"ele");
      MBXMLUtils::TiXmlText *text = new MBXMLUtils::TiXmlText(toStr(A.e(i,j)));
      elej->LinkEndChild(text);
      elei->LinkEndChild(elej);
    }
    ele->LinkEndChild(elei);
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
inline T fromMatStr(const std::string &str) {
  return T(str.c_str());
}

template < >
inline double fromMatStr(const std::string &str) {
  fmatvec::Mat A(str.c_str());
  return A(0,0);
}

template <class T>
void addElementText(MBXMLUtils::TiXmlElement *parent, std::string name, const T &value) {
  parent->LinkEndChild(new MBXMLUtils::TiXmlElement(name))->LinkEndChild(toXML(value));
}

class Deprecated {
  public:
    /*! register a deprecated feature with name message.
     * If e is NULL a stack trace is printed if available if e it not NULL MBXMLUtils::TiXml_location is printed. */
    static void registerMessage(const std::string &message, MBXMLUtils::TiXmlElement *e=NULL);
  private:
    static void printAllMessages();
    static std::set<std::vector<std::string> > allMessages;
    static bool atExitRegistred;
};

#endif

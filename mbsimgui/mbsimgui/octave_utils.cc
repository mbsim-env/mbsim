/* Copyright (C) 2004-2015 MBSim Development Team
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

#include <config.h>
#include <iostream>
#include "octave_utils.h"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

using namespace std;

namespace MBSimGUI {

  void OctaveElement::toStream(ostream &os) const {
    os << "# name: " << name << endl;
    os << "# type: " << type << endl;
  }

  void OctaveStruct::toStream(ostream &os) const {
    OctaveElement::toStream(os);
    for(auto i : ele)
      i->toStream(os);
  }

  OctaveElement* OctaveStruct::find(const string &name) {
    for(auto & i : ele)
      if(i->getName() == name)
        return i;
    return nullptr;
  }

  void OctaveScalar::toStream(ostream &os) const {
    OctaveElement::toStream(os);
    os << setw(28) << a << endl;
    os << endl;
  }

  void OctaveMatrix::toStream(ostream &os) const {
    OctaveElement::toStream(os);
    os << "# rows: " << A.rows() << endl;
    os << "# columns: " << A.cols() << endl;
    for(int i=0; i<A.rows(); i++) {
      for(int j=0; j<A.cols(); j++)
        os << setw(28) << A.e(i,j) << " ";
      os << endl;
    }
    os << endl;
  }

  void OctaveComplexMatrix::toStream(ostream &os) const {
    OctaveElement::toStream(os);
    os << "# rows: " << A.rows() << endl;
    os << "# columns: " << A.cols() << endl;
    for(int i=0; i<A.rows(); i++) {
      for(int j=0; j<A.cols(); j++)
        os << setw(28) << A.e(i,j) << " ";
      os << endl;
    }
    os << endl;
  }

  void OctaveCell::toStream(ostream &os) const {
    OctaveElement::toStream(os);
    os << "# rows: " << ele.size() << endl;
    os << "# columns: " << ele[0].size() << endl;
    for(const auto & i : ele)
      for(unsigned int j=0; j<i.size(); j++)
        i[j]->toStream(os);
    os << endl;
  }

  void OctaveParser::parse() {
    string str;
    streampos pos;
    do {
      pos = is.tellg();
      getline(is,str);
    } while(str.find("name")==string::npos);
    is.seekg(pos);
    OctaveElement* el = parseElement();
    while(el) {
      ele.push_back(el);
      el = parseElement();
    }
  }

  string OctaveParser::readName() {
    string buf;
    is >> buf >> buf >> buf;
    return buf;
  }

  string OctaveParser::readType() {
    string buf;
    is >> buf >> buf >> buf;
    string name = buf;
    getline(is,buf);
    name += buf;
    return name;
  }

  OctaveScalar* OctaveParser::readScalar() {
    string buf;
    is >> buf;
    auto a = boost::lexical_cast<double>(boost::algorithm::trim_copy(buf));
    return new OctaveScalar("",a);
  }

  OctaveMatrix* OctaveParser::readMatrix() {
    string buf;
    is >> buf >> buf >> buf;
    auto m = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    is >> buf >> buf >> buf;
    auto n = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    fmatvec::MatV A(m,n,fmatvec::NONINIT);
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++) {
        is >> buf;
        A(i,j) = boost::lexical_cast<double>(boost::algorithm::trim_copy(buf));
      }
    }
    return new OctaveMatrix("",A);
  }

  OctaveMatrix* OctaveParser::readDiagonalMatrix() {
    string buf;
    is >> buf >> buf >> buf;
    auto m = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    is >> buf >> buf >> buf;
    auto n = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    fmatvec::MatV A(m,n,fmatvec::INIT,0.);
    for(int i=0; i<m; i++) {
      is >> buf;
      A(i,i) = boost::lexical_cast<double>(boost::algorithm::trim_copy(buf));
    }
    return new OctaveMatrix("",A);
  }

  OctaveComplexMatrix* OctaveParser::readComplexMatrix() {
    string buf;
    is >> buf >> buf >> buf;
    auto m = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    is >> buf >> buf >> buf;
    auto n = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,complex<double> > A(m,n,fmatvec::NONINIT);
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++) {
        is >> buf;
        int pos = buf.find(',');
        A(i,j) = complex<double>(boost::lexical_cast<double>(boost::algorithm::trim_copy(buf.substr(1,pos-1))),boost::lexical_cast<double>(boost::algorithm::trim_copy(buf.substr(pos+1,buf.length()-pos-2))));
      }
    }
    return new OctaveComplexMatrix("",A);
  }

  OctaveCell* OctaveParser::readCell() {
    string buf;
    is >> buf >> buf >> buf;
    auto m = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    is >> buf >> buf >> buf;
    auto n = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    OctaveCell *cell = new OctaveCell("",m,n);
    for(int i=0; i<m; i++) {
      for(int j=0; j<n; j++)
        cell->setElement(i,j,parseElement());
    }
    return cell;
  }

  OctaveStruct* OctaveParser::readStruct() {
    string buf;
    is >> buf >> buf >> buf;
    is >> buf >> buf;
    is >> buf >> buf >> buf;
    auto n = boost::lexical_cast<int>(boost::algorithm::trim_copy(buf));
    OctaveStruct *strct = new OctaveStruct("",n);
    for(int i=0; i<n; i++)
      strct->setElement(i,parseElement());
    return strct;
  }

  OctaveElement* OctaveParser::parseElement() {
    string name = readName();
    string type = readType();
    OctaveElement *ele = nullptr;
    if(type == "scalar")
      ele = readScalar();
    else if(type == "matrix")
      ele = readMatrix();
    else if(type == "diagonal matrix")
      ele = readDiagonalMatrix();
    else if(type == "complex matrix")
      ele = readComplexMatrix();
    else if(type == "cell")
      ele = readCell();
    else if(type == "scalar struct")
      ele = readStruct();
    if(ele) {
      ele->setName(name);
      ele->setType(type);
    }
    return ele;
  }
  
  OctaveElement* OctaveParser::find(const string &name) {
    for(auto & i : ele)
      if(i->getName() == name)
        return i;
    return nullptr;
  }

}



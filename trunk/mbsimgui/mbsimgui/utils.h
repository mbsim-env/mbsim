/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef _UTILS_H_
#define _UTILS_H_

#include <QIcon>
#include <string>
#include <iostream>
#include <iomanip>
#include <limits>
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"

class QTreeWidgetItem;

/** Utilitiy class */
class Utils {
  public:
    // INITIALIZATION

    /** initialize the Utils class. Must be called before any member is used. */
    static void initialize();



    // HELPER FUNCTIONS

    /** Use QIconCached(filename) instead of QIcon(filename) everywhere
     * to cache the parsing of e.g. SVG files. This lead to a speedup
     * (at app init) by a factor of 11 in my test case. */
    static const QIcon& QIconCached(const QString& filename);

//    /** Convenienc function to convert cardan angles to a rotation matrix */
//    static SbRotation cardan2Rotation(const SbVec3f& c);
//    /** Convenienc function to convert a rotation matrix to cardan angles */
//    static SbVec3f rotation2Cardan(const SbRotation& r);



  private:
    // INITIALIZATION
    static bool initialized;
};

inline std::string toStr(const std::string &str) {
  return str;
}

inline std::string toStr(int i) {
  std::stringstream s;
  s << i;
  return s.str();
}

inline std::string toStr(double d) {
  std::stringstream s;
  s << d;
  return s.str();
}

template <class AT>
inline std::string toStr(const std::vector<AT> &x) {
  if(x.size()==0)
    return "";
  std::stringstream s;
  s << "[";
  for(int i=0; i<x.size(); i++) {
    s << std::setprecision(std::numeric_limits<double>::digits10) << x[i];
    if(i<x.size()-1)
      s << ";";
  }
  s << "]";
  return s.str();
}

template <class AT>
inline std::string toStr(const std::vector<std::vector<AT> > &A) {
  if(A.size()==0 || A[0].size()==0)
    return "";
  std::stringstream s;
  s << "[";
  for(int i=0; i<A.size(); i++) {
    for(int j=0; j<A[i].size(); j++) {
      s << std::setprecision(std::numeric_limits<double>::digits10) << A[i][j];
      if(j<A[i].size()-1)
        s << ",";
    }
    if(i<A.size()-1)
      s << ";";
  }
  s << "]";
  return s.str();
}

template <class AT>
inline std::string toStr2(const std::vector<std::vector<AT> > &A) {
  std::stringstream s;
  s << "[\n";
  for(int i=0; i<A.size(); i++) {
    for(int j=0; j<A[i].size(); j++) {
      s << std::setprecision(std::numeric_limits<double>::digits10) << A[i][j];
      if(j<A[i].size()-1)
        s << " ";
    }
    s << "\n";
  }
  s << "]";
  return s.str();
}

inline std::vector<std::string> extract(const std::string &str, char c) {
  std::vector<int> vi;
  vi.push_back(-1);
  int i=0;
  while(true) {
    i = str.find(c,i); 
    if(i!=-1)
      vi.push_back(i);
    else
      break;
    i+=1;
  } 
  vi.push_back(str.size());
  std::vector<std::string> ret(vi.size()-1);
  for(unsigned int i=0; i<vi.size()-1; i++) {
    ret[i] = str.substr(vi[i]+1,vi[i+1]-vi[i]-1);
  }
  return ret;
}

inline std::vector<std::string> strToSVec(const std::string &str) {
  if(str=="") {
    std::vector<std::string> x;
    return x;
  }
  int pos1 = str.find("["); 
  int pos2 = str.find("]"); 
  std::string str0 = str.substr(pos1+1,pos2-1);
  std::vector<std::string> str1 = extract(str0,';');
  std::vector<std::string> x(str1.size());
  for(unsigned int i=0; i<str1.size(); i++) {
    x[i] = str1[i];
  }
  return x;
}

inline std::vector<std::vector<std::string> > strToSMat(const std::string &str) {
  if(str=="") {
    std::vector<std::vector<std::string> > A;
    return A;
  }
  int pos1 = str.find("["); 
  int pos2 = str.find("]"); 
  std::string str0 = str.substr(pos1+1,pos2-1);
  std::vector<std::string> str1 = extract(str0,';');
  std::vector<std::vector<std::string> > A(str1.size());
  for(unsigned int i=0; i<str1.size(); i++) {
    std::vector<std::string> str2 = extract(str1[i],',');
    A[i].resize(str2.size());
    for(unsigned int j=0; j<str2.size(); j++)
      A[i][j] = str2[j];
  }
  return A;
}

inline std::vector<std::vector<double> > strToDMat(const std::string &str) {
  std::istringstream iss(str);
  char c;
  iss>>c;
  if(c=='[') iss.str(str);
  else iss.str(std::string("[")+str+"]");

  int m=0,n=0;
  int buf=0;
  iss >> c;
  double x;
  do {
    iss >> x;
    iss >> c;
    if(c==';') {
      if(buf)
        assert(buf == n);

      buf=n;
      n=0;
      m++;
    }
    else if(c==',')
      n++;
    c='0';
  } while(iss);

  n++; m++;

  iss.clear();
  iss.seekg(0);
  iss >> c;
  std::vector<std::vector<double> > A(m);
  for(int i=0; i<m; i++) {
    A[i].resize(n);
    for(int j=0; j<n; j++) {
      iss >> A[i][j]; 
      iss >> c;
    }
  }
  return A;
}

template <class T>
void addElementText(TiXmlElement *parent, std::string name, T value) {
  std::ostringstream oss;
  oss << std::setprecision(std::numeric_limits<double>::digits10) << toStr(value);
  parent->LinkEndChild(new TiXmlElement(name))->LinkEndChild(new TiXmlText(oss.str()));
}

template <class T>
void addElementAttributeAndText(TiXmlElement *parent, std::string name, std::string attribute, std::string attributeName, T value) {
  std::ostringstream oss;
  oss << std::setprecision(std::numeric_limits<double>::digits10) << toStr(value);
  TiXmlElement* ele = new TiXmlElement(name);
  ele->SetAttribute(attribute,attributeName);
  ele->LinkEndChild(new TiXmlText(oss.str()));
  parent->LinkEndChild(ele);
}

std::vector<std::vector<double> > Cardan2AIK(const std::vector<std::vector<double> > &x);
std::vector<std::vector<double> > AIK2Cardan(const std::vector<std::vector<double> > &x);

 /**
   * \brief basic event class for MBSim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimEvent {
    public:
      /**
       * \brief constructor
       */
      MBSimEvent() {}
      
      /**
       * \brief destructor
       */
      virtual ~MBSimEvent() {}
  };

  /**
   * \brief basic exception interface for mbsim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimException : public MBSimEvent{
    public:
      /**
       * \brief constructor
       */
      MBSimException() : MBSimEvent() {}
      
      /**
       * \brief destructor
       */
      virtual ~MBSimException() {}

      /* INTERFACE */
      /**
       * \brief prints exception message on stdout
       */
      virtual void printExceptionMessage() = 0; 
  };

  /**
   * \brief basic error class for mbsim
   * \author Thorsten Schindler
   * \date 2009-03-20 inital commit (Thorsten Schindler)
   */
  class MBSimError : public MBSimException {
    public:
      /**
       * \brief constructor
       * \param message to be written
       */
      MBSimError(const std::string &mbsim_error_message_); 
      
      /**
       * \brief destructor
       */
      virtual ~MBSimError() {}

      /* INHERITED INTERFACE */
      virtual void printExceptionMessage();

    private:
      /**
       * \brief error message
       */
      std::string mbsim_error_message;

  };

std::string evaluateOctave(const std::string &program);

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
 
QTreeWidgetItem* getChild(QTreeWidgetItem *parentItem, const QString &str);

template <class AT>
inline std::vector<std::vector<AT> > transpose(const std::vector<std::vector<AT> > &A) {
 std::vector<std::vector<AT> > B(A[0].size());
 for(int i=0; i<B.size(); i++) {
   B[i].resize(A.size());
   for(int j=0; j<B[i].size(); j++) {
     B[i][j] = A[j][i];
   }
 }
 return B;
}

template <class AT>
inline std::vector<AT> getScalars(int m, const AT &d) {
  std::vector<AT> x(m);
  for(int i=0; i<m; i++) {
    x[i] = d;
  }
  return x;
}

template <class AT>
inline std::vector<std::vector<AT> > getScalars(int m, int n, const AT &d) {
  std::vector<std::vector<AT> > A(m);
  for(int i=0; i<m; i++) {
    A[i].resize(n);
    for(int j=0; j<n; j++)
      A[i][j] = d;
  }
  return A;
}

template <class AT>
inline std::vector<std::vector<AT> > getEye(int m, int n, const AT &d, const AT &z) {
  std::vector<std::vector<AT> > A(m);
  for(int i=0; i<m; i++) {
    A[i].resize(n);
    for(int j=0; j<n; j++)
      A[i][j] = z;
    A[i][i] = d;
  }
  return A;
}

template <class AT>
inline std::vector<std::vector<AT> > getMat(int m, int n, const AT &d) {
  std::vector<std::vector<AT> > A(m);
  for(int i=0; i<m; i++) {
    A[i].resize(n);
    for(int j=0; j<n; j++)
      A[i][j] = d;
  }
  return A;
}

inline QStringList noUnitUnits() {
  QStringList units;
  units << "" << "-" << "%";
  return units;
}

inline QStringList lengthUnits() {
  QStringList units;
  units << "mum" << "mm" << "cm" << "dm" << "m" << "km";
  return units;
}

inline QStringList angleUnits() {
  QStringList units;
  units << "rad" << "degree";
  return units;
}

inline QStringList massUnits() {
  QStringList units;
  units << "mg" << "g" << "kg" << "t";
  return units;
}

inline QStringList inertiaUnits() {
  QStringList units;
  units << "g*mm^2" << "kg*mm^2" << "kg*m^2";
  return units;
}

inline QStringList accelerationUnits() {
  QStringList units;
  units << "m/s^2"; 
  return units;
}

inline QStringList stiffnessUnits() {
  QStringList units;
  units << "N/mm" << "N/m"; 
  return units;
}

inline QStringList dampingUnits() {
  QStringList units;
  units << "N*s/m"; 
  return units;
}

inline QStringList forceUnits() {
  QStringList units;
  units << "mN" << "N" << "kN";
  return units;
}

#endif

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

#ifndef OCTAVE_UTILS_H_
#define OCTAVE_UTILS_H_

#include <string>
#include <vector>
#include <fmatvec/fmatvec.h>

namespace MBSim {

  class OctaveElement {
    protected:
      std::string name;
      std::string type;
    public:
      OctaveElement(const std::string &name_, const std::string &type_) : name(name_), type(type_) { }
      virtual ~OctaveElement() { }
      const std::string& getName() const { return name; }
      const std::string& getType() const { return type; }
      void setName(const std::string &name_) { name = name_; }
      void setType(const std::string &type_) { type = type_; }
      virtual void toStream(std::ostream &os=std::cout) const;
      virtual OctaveElement* find(const std::string &name) { return 0; }
  };

  class OctaveStruct : public OctaveElement {
    private:
      std::vector<OctaveElement*> ele;
    public:
      OctaveStruct(const std::string &name, int m) : OctaveElement(name,"scalar struct"), ele(m) { }
      ~OctaveStruct() {
        for(unsigned int i=0; i<ele.size(); i++)
          delete ele[i];
      }
      int getNumberOfElements() const { return ele.size(); }
      void setElement(int i, OctaveElement *ele_) { ele[i] = ele_; }
      const OctaveElement* getElement(int i) const { return ele[i]; }
      void toStream(std::ostream &os=std::cout) const;
      OctaveElement* find(const std::string &name);
  };

  class OctaveScalar : public OctaveElement {
    private:
      double a;
    public:
      OctaveScalar(const std::string &name, double a_) : OctaveElement(name,"scalar"), a(a_) { }
      void toStream(std::ostream &os=std::cout) const;
      double get() const { return a; }
  };

  class OctaveMatrix : public OctaveElement {
    private:
      fmatvec::MatV A;
    public:
      OctaveMatrix(const std::string &name, const fmatvec::MatV &A_) : OctaveElement(name,"matrix"), A(A_) { }
      void toStream(std::ostream &os=std::cout) const;
      template <class T>
      T get() const { return T(A); }
  };

  class OctaveComplexMatrix : public OctaveElement {
    private:
      fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,std::complex<double> > A;
    public:
      OctaveComplexMatrix(const std::string &name, const fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,std::complex<double> > &A_) : OctaveElement(name,"complex matrix"), A(A_) { }
      void toStream(std::ostream &os=std::cout) const;
      template <class T> T get() const { return T(A); }
  };

  class OctaveCell : public OctaveElement {
    private:
      std::vector<std::vector<OctaveElement*> > ele;
    public:
      OctaveCell(const std::string &name, int m, int n) : OctaveElement(name,"cell"), ele(m) {
        for(int i=0; i<m; i++)
          ele[i].resize(n);
      }
      ~OctaveCell() {
        for(unsigned int i=0; i<ele.size(); i++)
          for(unsigned int j=0; j<ele[i].size(); j++)
            delete ele[i][j];
      }
      int getNumberOfRows() const { return ele.size(); }
      int getNumberOfColumns() const { return ele[0].size(); }
      void setElement(int i, int j, OctaveElement *ele_) { ele[i][j] = ele_; }
      const OctaveElement* getElement(int i, int j) const { return ele[i][j]; }
      template <class T>
        std::vector<std::vector<T> > get() const { 
          std::vector<std::vector<T> > A(ele.size());
          for(unsigned int i=0; i<ele.size(); i++) {
            for(unsigned int j=0; j<ele[i].size(); j++) {
              OctaveMatrix *octMat = dynamic_cast<OctaveMatrix*>(ele[i][j]);
              if(octMat) 
                A[i].push_back(T(octMat->get<T>()));
              else 
                A[i].push_back(T());
            }
          }
          return A;
        } 
      void toStream(std::ostream &os=std::cout) const;
  };

  class OctaveParser {
    private:
      std::ifstream is;

    public:
      OctaveParser(const std::string &file) {
        is.open(file.c_str());
      }

      ~OctaveParser() {
        is.close();
      }

      bool fileOpen() const { return is.is_open(); }

      std::vector<OctaveElement*> parse();
      std::string readName();
      std::string readType();
      OctaveScalar* readScalar();
      OctaveMatrix* readMatrix();
      OctaveMatrix* readDiagonalMatrix();
      OctaveComplexMatrix* readComplexMatrix();
      OctaveCell* readCell();
      OctaveStruct* readStruct();
      OctaveElement* parseElement();
  };

  OctaveElement* find(const std::vector<OctaveElement*> &ele, const std::string &name);

}

#endif

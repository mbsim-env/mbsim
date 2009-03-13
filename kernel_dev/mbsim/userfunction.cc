/* Copyright (C) 2004-2006  Martin Förg, Felix Kahr
 
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
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */
#include <config.h>
#include <mbsim/userfunction.h> 
#include <mbsim/utils/ppolynom.h> 
#include <fstream>

namespace MBSim {

  void FuncTable::setXY(const Vec& X,const Mat& Y) {
    int cols=1+Y.cols();
    int rows=X.rows();
    assert(rows==Y.rows());
    table.resize(rows,cols);
    table(Index(0,rows-1),Index(0,0))=X;
    table(Index(0,rows-1),Index(1,cols-1))=Y;
  }

  void FuncTable::setFile(const string &filename) {
    char dummy[100000];
    int rows = 0;
    {
      ifstream is(filename.c_str());
      while(is.good()) {
	is.getline(dummy,100000);
	rows ++;
      } 
      is.close();
    }
    rows--;

    table.resize(rows,2);

    ifstream is(filename.c_str());
    for(int i=0; i<rows; i++) {
      is >> table(i,0) >> table(i,1);
    } 
    is.close();
  }


  Vec FuncTable::operator()(double x) {
    Vec res(1);
    if(x<=table(0,0)) {
      res(0) = table(0,1);
      oldi=0;
      return res;
    }
    if(x>=table(table.rows()-1,0)){
      res(0) = table(table.rows()-1,1);
      oldi=table.rows()-2;
      return res;
    }

    int nr=table.rows();
    for(int i=0;i<(nr)/2;i++){
      int i1=(oldi+i+1)%(nr-1); 
      int i2=(oldi-i)%(nr-1);
      //cout << "\ni: " << i  << endl;
      //cout << "i1=" << i1  << endl;
      //cout << "i2=" << i2  << endl;
      if(i2<0) i2+=(nr-1);
      if(x > table(i1,0) && x <= table(i1+1,0)) {
	double delta_y = (table(i1+1,1)-table(i1,1));
	double delta_x = (table(i1+1,0)-table(i1,0));
	double m = delta_y/delta_x;
	res(0) = table(i1,1) + m*(x-table(i1,0));
	oldi=i1;
	return res;
      } 
      if(x > table(i2,0) && x <= table(i2+1,0)) {
	double delta_y = (table(i2+1,1)-table(i2,1));
	double delta_x = (table(i2+1,0)-table(i2,0));
	double m = delta_y/delta_x;
	res(0) = table(i2,1) + m*(x-table(i2,0));
	oldi=i2;
	return res;
      } 
    }
    cout << "Vec FuncTable::operator()(double x) did not offer return value" << endl;
    throw 1;
    return 0;
  }

  //TODO ableiten
  void FuncTablePer::setFile(const string &filename) {
    char dummy[100000];
    int rows = 0;
    {
      ifstream is(filename.c_str());
      while(is.good()) {
	is.getline(dummy,100000);
	rows ++;
      } 
      is.close();
    }
    rows--;

    table.resize(rows,2);

    ifstream is(filename.c_str());
    for(int i=0; i<rows; i++) {
      is >> table(i,0) >> table(i,1);
    } 
    is.close();
  }  

  Vec FuncTablePer::operator()(double x) {
    Vec res(1);
    while(x<table(0,0)) {
      res(0) = x+table(table.rows(),0);
    }
    while(x>table(table.rows(),0)) {
      res(0) = x-table(table.rows(),0);
    }

    for(int i=0; i<table.rows()-1; i++) {
      if(x == table(i,0)) {
	res(0) = table(i,1);
	return res;
      }
      if(x > table(i,0) && x <= table(i+1,0)) {

	double delta_y = (table(i+1,1)-table(i,1));
	double delta_x = (table(i+1,0)-table(i,0));
	double m = delta_y/delta_x;

	res(0) = table(i,1) + m*(x-table(i,0));
	return res;
      }
    }
    int i = table.rows()-1;
    res(0) = table(i,1);
    return res;
  }

  FuncSum::~FuncSum() { 
    vector<DataInterfaceBase*>::iterator i;
    for(i =  dib.begin(); i != dib.end(); ++i)
      delete *i;
  }

  void FuncSum::addInput(DataInterfaceBase* dib_,double c_,int dim){
    if(!dib.size()) {
      outputdim=dim;
    }
    if(outputdim==dim)
    {
      dib.push_back(dib_);
      c.push_back(c_);
    }
    else {cout << "Error: FuncSum: Only DataInterfaceBase with the same Vector size can be added." << endl; throw 50; }
  }

  void FuncSum::addInput(DataInterfaceBase* dib_,double c_){
    //TODO: geht nicht mehr wegen SPSys
    if(!dib.size()) {
      outputdim=(*dib_)(0).size();
    }
    if(outputdim==(*dib_)(0).size())
    {
      dib.push_back(dib_);
      c.push_back(c_);
    }
    else {cout << "Error: FuncSum: Only DataInterfaceBase with the same Vector size can be added." << endl; throw 50; }
  }

  Vec FuncSum::operator()(double x) {
    Vec y(outputdim,INIT,0.0);
    for(unsigned int i=0;i<dib.size();i++)
    {
      y+=c[i]*(*dib[i])(x);
    }
    return y;
  }  
  FuncConst::FuncConst(const Vec& c_) : UserFunction() {
    c=c_;
  }
  FuncLinear::FuncLinear(const Vec& a_,const Vec& b_) : UserFunction() {
    a=a_;
    b=b_;
    if(a.size()!=b.size()) {cout << "Error: FuncLinear: a and b have different dimensions" << endl; throw 50; }
  }
  FuncGainOffset::FuncGainOffset(DataInterfaceBase *f_,double gain,const Vec& offset) : UserFunction() {
    f=f_;
    a=gain;
    b=offset;
  }
  FuncFunction::FuncFunction(UserFunction *f_, DataInterfaceBase *g_) : UserFunction() {
    f=f_;
    g=g_;
  }

}

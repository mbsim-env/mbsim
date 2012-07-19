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

#endif

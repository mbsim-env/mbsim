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

#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"

#include <mbsim/utils/eps.h>

using namespace fmatvec;

std::string numtostr(int i){
  std::ostringstream oss;
  oss << i;
  return oss.str(); 
}

std::string numtostr(double d) {
  std::ostringstream oss;
  oss << d;
  return oss.str(); 
}

double degtorad(double alpha) {return alpha/180.*M_PI; }
double radtodeg(double phi) {return phi/M_PI*180.; }
fmatvec::Vec degtorad(fmatvec::Vec alpha) {return alpha/180.*M_PI; }
fmatvec::Vec radtodeg(fmatvec::Vec phi) {return phi/M_PI*180.; }

double sign(double x) {
  if(x>0)
    return 1.0;
  else if(x<0)
    return -1.0;
  else 
    return 0;
}

int min(int i, int j) {
  return i<j?i:j;
}  

Vec tildetovec(const SqrMat &A) {
  Vec x(3,NONINIT);
  x(0) = A(2,1);
  x(1) = A(0,2);
  x(2) = A(1,0);
  return x;
}

double ArcTan(double x, double y) {
  double phi;
  phi = atan2(y, x);

  if (phi < -MBSim::macheps())
    phi += 2 * M_PI;
  return phi;
}

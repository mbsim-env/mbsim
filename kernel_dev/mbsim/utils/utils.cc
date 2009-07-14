/* Copyright (C) 2004-2006  Martin Förg

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

#include "utils.h"

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

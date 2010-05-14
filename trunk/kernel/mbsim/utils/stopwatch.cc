/* Copyright (C) 2004-2009 MBSim Development Team

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
 *   rhuber@users.berlios.de
 *
 */

#include <config.h>
#ifdef HAVE_SYS_TIME_H 
#include <sys/time.h>
#endif
#include "mbsim/utils/stopwatch.h"
#include <time.h>
#include <cmath>
#include <iostream>

using namespace std;

namespace MBSim {

  StopWatch::StopWatch() { 
#ifdef HAVE_SYS_TIME_H  
    begin_tv = new timeval;
    end_tv = new timeval;
#endif
  }
  StopWatch::~StopWatch() {
#ifdef HAVE_SYS_TIME_H  
    delete begin_tv;
    delete end_tv;
#endif
  }

  //================================================================================================
  /* More precise Version (1e-6s), but gettimeofday(..) is not avialable on all platforms */
#ifdef HAVE_SYS_TIME_H  
  void StopWatch::start() {
    gettimeofday(begin_tv, 0);
  }

  double StopWatch::stop() {
    gettimeofday(end_tv, 0);
    double IntTime;
    IntTime = (end_tv->tv_sec -begin_tv->tv_sec);
    IntTime += end_tv->tv_usec*1.0/1000000.0 -begin_tv->tv_usec*1.0/1000000.0;
    *begin_tv = *end_tv;
    return IntTime;
  }
#endif
  //===============================================================================================

  /* Portabel Version with clock() (not as precise as gettimeofday(); 1e-2s ?)
   * 
   * used, if sys/time.h is not found
   */ 
#ifndef HAVE_SYS_TIME_H 
  void StopWatch::start() {
    begin_d = clock();
  }

  double StopWatch::stop() {
    end_d = clock();
    double IntTime = end_d - begin_d;
    IntTime /= CLOCKS_PER_SEC;
    begin_d=end_d;
    return IntTime;
  }
#endif 
  //================================================================================================

}

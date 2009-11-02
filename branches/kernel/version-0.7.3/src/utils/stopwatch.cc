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
 *   rhuber@users.berlios.de
 *
 */
#include <config.h>
#include <sys/time.h>
#include "stopwatch.h"
#include <cmath>

using namespace std;

namespace MBSim {

  void StopWatch::start() {
    gettimeofday(&begin, 0);
  }

  double StopWatch::stop() {
    gettimeofday(&end, 0);
    double IntTime;
    IntTime = (end.tv_sec -begin.tv_sec);
    IntTime += end.tv_usec*1.0/1000000.0 -begin.tv_usec*1.0/1000000.0;
    begin=end;
    return IntTime;
  }

}


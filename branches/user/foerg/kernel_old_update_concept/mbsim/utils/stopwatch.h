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
 */
#ifndef _STOPWATCH_H_
#define _STOPWATCH_H_

/*! 
 * \brief class for time measurement (timer, stopwatch) 
 * \author Robert Huber
 * \date 2010-05-11 portable version without gettimeofday added (if sys/time.h is not availible) (Robert Huber)
 * \date 2010-07-06 flag in StopWatch::stop to reset timer (Robert Huber)
 */

#ifdef HAVE_SYS_TIME_H 
struct timeval;
#endif

namespace MBSim {

  class StopWatch  {

    protected:

#ifdef HAVE_SYS_TIME_H 
      timeval *begin_tv;
      timeval *end_tv;
#else
      double begin_d;
      double end_d;      
#endif

    public:
      StopWatch();
      ~StopWatch();

      /* \brief starts timer */
      void start();
      /* \brief stops timer and returns enclosed time
       * \param reset if flag = true clock is reseted
       */
      double stop(bool reset=true);

  };
}
#endif


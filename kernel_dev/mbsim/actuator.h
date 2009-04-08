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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "mbsim/link_mechanics.h"

namespace MBSim {

  class DataInterfaceBase; 

  /*!
   * \brief predefines load between two frames with additional possibility of rotation
   * \author Martin Foerg
   * \date 2009-04-06 LinkMechanics added (Thorsten Schindler)
   * \todo remove setUserfunction TODO
   * \todo visualisation TODO
   */
  class Actuator : public LinkMechanics {
    public:
      /**
       * \brief constructor
       * \param name
       */
      Actuator(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Actuator();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateh(double t);
      virtual void updateg(double t) {}
      virtual void updategd(double t) {}
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init();
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void calclaSize();
      virtual bool isActive() const { return true; }
      virtual bool gActiveChanged() { return false; }
      /***************************************************/

      /* GETTER / SETTER */
      void setKOSY(int id) { KOSYID = id; assert(KOSYID >= 0); assert(KOSYID <= 2); }
      void setSignal(DataInterfaceBase *func_) { func = func_; }
      /***************************************************/

      /**
       * \param first frame to connect
       * \param second frame to connect
       */
      void connect(FrameInterface *frame1, FrameInterface *frame2);
      
      /**
       * \param local force direction
       */
      void setForceDirection(const fmatvec::Mat& fd);
      
      /**
       * \param local moment direction
       */
      void setMomentDirection(const fmatvec::Mat& md);

      /**
       * \brief initialises signals describing norm of force / moment
       * \param parent, where to add the signal
       */
      void initDataInterfaceBase(DynamicSystemSolver *parentds);

    protected:
      /**
       * \brief indices of forces and moments
       */
      fmatvec::Index IT, IR;

      /**
       * \brief local force and moment direction
       */
      fmatvec::Mat forceDir, momentDir; 

      /**
       * \brief global force and moment direction
       */
      fmatvec::Mat Wf, Wm; 

      /**
       * \brief global force and moment direction for each body
       */
      fmatvec::Mat WF[2], WM[2]; 

      /**
       * \brief force / moment norm function
       */
      DataInterfaceBase *func;

      /**
       * \brief frame index for rotating forces
       */
      int KOSYID;

  };
}

#endif


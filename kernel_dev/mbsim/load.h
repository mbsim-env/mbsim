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

#ifndef _LOAD_H_
#define _LOAD_H_

#include <mbsim/link_mechanics.h>

namespace MBSim {

  class DataInterfaceBase;

  /*! 
   * \brief predefines load for one frame with additional possibility of rotation
   * \author Martin Foerg
   * \date 2009-04-06 LinkMechanics added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \todo delete and include functionality in actuator TODO
   */
  class Load : public LinkMechanics {
    public: 
      /**
       * \brief constructor
       * \param name
       */
      Load(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Load();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      void updateh(double t);
      void updateg(double t) {}
      void updategd(double t) {}
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      void init();
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      void calclaSize();
      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      /***************************************************/

      /* GETTER / SETTER */
      void setKOSY(int id) { KOSYID = id; assert(KOSYID >= 0); assert(KOSYID <= 1); }
      void setSignal(DataInterfaceBase *func_) { func = func_; }
      /***************************************************/

      /**
       * \param frame to connect
       */
      void connect(Frame *frame1);

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


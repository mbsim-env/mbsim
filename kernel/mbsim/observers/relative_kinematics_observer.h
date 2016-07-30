/* Copyright (C) 2004-2016 MBSim Development Team
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
 * Contact: martin.o.foerg@gmail.com
 */

#ifndef _RELATIVE_KINEMATICS_OBSERVER_H__
#define _RELATIVE_KINEMATICS_OBSERVER_H__

#include "mbsim/observers/kinematics_observer.h"

namespace MBSim {

  class RelativeKinematicsObserver : public KinematicsObserver {
    private:
      Frame* refFrame;
      std::string saved_frameOfReference;
#ifdef HAVE_OPENMBVCPPINTERFACE
      std::shared_ptr<OpenMBV::Arrow> openMBVrTrans, openMBVrRel;
      std::shared_ptr<OpenMBV::Arrow> openMBVvTrans, openMBVvRot, openMBVvRel, openMBVvF;
      std::shared_ptr<OpenMBV::Arrow> openMBVaTrans, openMBVaRot, openMBVaZp, openMBVaCor, openMBVaRel, openMBVaF;
      std::shared_ptr<OpenMBV::Arrow> openMBVomTrans, openMBVomRel;
      std::shared_ptr<OpenMBV::Arrow> openMBVpsiTrans, openMBVpsiRot, openMBVpsiRel;
#endif

    public:
      RelativeKinematicsObserver(const std::string &name="");
      void setFrameOfReference(Frame *frame_) { refFrame = frame_; }

      void init(InitStage stage);
      virtual void plot();
      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}  

#endif

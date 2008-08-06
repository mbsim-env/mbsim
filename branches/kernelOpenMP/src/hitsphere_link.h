/* Copyright (C) 2006  Martin Förg, Roland Zander
 
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
 *   rzander@users.berlios.de
 *
 */

#ifndef _HITSPHERE_LINK_H_
#define _HITSPHERE_LINK_H_

#include<string>
#include "object.h"
#include "link.h"

namespace MBSim {

  class MultiBodySystem;

  /*! \brief Hit spheres for all bodies including all contours
   * 
   *  Used for pre-identification of pairings with potential contacts
   */
  class HitSphereLink {
    protected:
      /** managed bodies */
      Object* obj[2];
      /** FLAG if hitspheres intersect */
      bool active;
      /** managed link list */
      vector<Link*> linkList;
      /** associated MBS */
      MultiBodySystem* mbs;

    public:
      /*! Constructor */
      HitSphereLink();
      /*! Destructor */
      virtual ~HitSphereLink() {}
      /*! Set managed bodies and links */
      void setParents(Object *obj0, Object *obj1, Link* link);
	  /*! Get involved objects */
      Object* getObject(const int &id) const {return obj[id];}
	  /*! Initialise hitspheres */
      void init();
      /*! Check if the hitspheres intersect */
      void checkActive();
      /*! Get FLAG for intersecting hitspheres */
      bool isActive() const {return active;}
      /*! Get managed links */
      vector<Link*> getLinkList();
  };

}

#endif /* _HITSPHERE_LINK_H_ */

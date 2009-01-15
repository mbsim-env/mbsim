/* Copyright (C) 2004-2006  Martin Förg, Roland Zander
 
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

#ifndef _CONTACT_H_
#define _CONTACT_H_

#include "link.h"
#include "contour.h"
#include "contour_pdata.h"

namespace MBSim {


  class ContactKinematics;
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;

  /*! \brief Class for contacts
   *
   * Basis class for Contacts between Contours, mainly implementing geometrical informations of ContactPairings
   *
   * */
  class Contact: public Link {

    protected:

      vector<unsigned int> gActive, gActive0;
      vector<unsigned int*> gdActive, gdActive0;

      /** index for tangential directions in projection matrices */
      Index iT;

      /** number of friction directions: 0 = frictionless, 1 = planar friction, 2 = spatial friction */
      //int nFric;

      ContactKinematics *contactKinematics;

      vector< ContourPointData* > cpData;

      double argN;
      Vec argT;

      void checkActive();

      GeneralizedForceLaw *fcl;
      FrictionForceLaw *fdf;
      GeneralizedImpactLaw *fnil;
      FrictionImpactLaw *ftil;

      Vec gdn, gdd;

      vector<Vec> gk, gdk, gdnk, gddk, lak, wbk, svk, rFactork;
      vector<Vector<int> > jsvk;
      vector<Mat*> fF;
      vector<Vec*> WF;
      vector<Mat*> Vk, Wk;
      vector<int> laSizek, laIndk, gSizek, gIndk, gdSizek, gdIndk, svSizek, svIndk, rFactorSizek, rFactorIndk;

    public:
      /*!
	\param name name of Contact
	\param setValued true, if force law is set-valued, else false for functional law
	*/      
      Contact(const string &name);
      Contact(const string &name, bool flag) : Link(name,flag) {}

      virtual ~Contact();

      bool isSetValued() const;

      void calcxSize();

      void calclaSize();
      void calcgSize();
      void calcgdSize();
      void calcrFactorSize();
      void calcsvSize();

      void load(const string& path, ifstream &inputfile);
      void save(const string &path, ofstream &outputfile);

      void solveImpactsFixpointSingle();
      void solveConstraintsFixpointSingle();
      void solveImpactsGaussSeidel();
      void solveConstraintsGaussSeidel();
      void solveImpactsRootFinding();
      void solveConstraintsRootFinding();
      void jacobianConstraints();
      void jacobianImpacts();

      /*geerbt*/
      void init();
      void preinit();

      /*! define wether HitSpheres are tested or ignored
      */
      void connectHitSpheres(Contour *contour1, Contour* contour2);

      /*! connect two Contour s
	\param contour1 first contour
	\param contour2 second contour
	*/
      void connect(Contour *contour1, Contour* contour2);

      void updateg(double t);
      void updategd(double t);
      void updater(double t);
      void updateW(double t);
      void updateV(double t);
      void updatewb(double t);
      void updateh(double t);
      void updaterFactors();
      void updateJacobians(double t);

      void resizeJacobians(int j); 

      void updatelaRef(const Vec& ref);
      void updategRef(const Vec& ref);
      void updategdRef(const Vec& ref);
      void updateWRef(const Mat &ref, int j=0);
      void updatewbRef(const Vec &ref);
      void updateVRef(const Mat &ref, int j=0);
      void updatehRef(const Vec &ref, int j=0);
      void updatesvRef(const Vec &ref);
      void updatejsvRef(const Vector<int> &ref);
      void updaterFactorRef(const Vec &ref);

      void updateStopVector(double t);

      void setContactForceLaw(GeneralizedForceLaw *fcl_) {fcl = fcl_;}
      void setContactImpactLaw(GeneralizedImpactLaw *fnil_) {fnil = fnil_;}
      void setFrictionForceLaw(FrictionForceLaw *fdf_) {fdf = fdf_;}
      void setFrictionImpactLaw(FrictionImpactLaw *ftil_) {ftil = ftil_;}

      virtual int getFrictionDirections(); 
      void setContactKinematics(ContactKinematics* ck) {contactKinematics = ck;}
      string getType() const {return "Contact";}

      bool gActiveChanged();
      bool isActive() const;

      void checkActiveg();
      void checkActivegd();
      void checkActivegdn();
      void checkActivegdd(); 
      void checkAllgd();

      void updateCondition();
      void checkConstraintsForTermination();
      void checkImpactsForTermination();

      using Link::connect;
  };

}

#endif

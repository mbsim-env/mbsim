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
 * Contact: thschindler@users.berlios.de
 */

#ifndef _CONTOUR1S_ANALYTICAL_H_
#define _CONTOUR1S_ANALYTICAL_H_

#include "mbsim/contours/contour1s.h"

namespace MBSim {

  class ContourFunction1s;
  class ContactKinematics;

  /** 
   * \brief analytical description of contours with one contour parameter
   * \author Robert Huber
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class Contour1sAnalytical : public Contour1s {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour1sAnalytical(const std::string &name) : Contour1s(name)
# ifdef HAVE_OPENMBVCPPINTERFACE
                                                     , openMBVRigidBody(0)
# endif
                                                     {}

      /**
       * \brief destructor
       */
      virtual ~Contour1sAnalytical();

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Contour1sAnalytical"; }
      virtual void plot(double t, double dt = 1);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual void updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff);
      virtual void updateJacobiansForFrame(ContourPointData &cp, int j=0);
      virtual void init(InitStage stage);
      virtual double computeCurvature(ContourPointData &cp);
      virtual double computeDistance(const double s, const int order=0);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOURCONTINUUM */
      virtual void computeRootFunctionPosition(ContourPointData &cp) { updateKinematicsForFrame(cp,position); }
      virtual void computeRootFunctionFirstTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,firstTangent); }
      virtual void computeRootFunctionNormal(ContourPointData &cp) { updateKinematicsForFrame(cp,normal); }
      virtual void computeRootFunctionSecondTangent(ContourPointData &cp) { updateKinematicsForFrame(cp,secondTangent); }
      /***************************************************/

      /* GETTER / SETTER */
      void setContourFunction1s(ContourFunction1s* f) { funcCrPC = f; }
      ContourFunction1s* getContourFunction1s() { return funcCrPC; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif
      
      virtual void initializeUsingXML(TiXmlElement *element);

      /**
       * \brief contact search for RigidContours
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      ContactKinematics * findContactPairingWith(std::string type0, std::string type1);

    protected:
      ContourFunction1s * funcCrPC;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::RigidBody *openMBVRigidBody;
#endif
  };

}

#endif /* _CONTOUR1S_ANALYTICAL_H_ */


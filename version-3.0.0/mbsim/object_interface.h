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

#ifndef _OBJECT_INTERFACE_H_
#define _OBJECT_INTERFACE_H_

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Group;
}
#endif

namespace MBSim {

  class DynamicSystem;

  /*!
   * \brief interface for objects for usage in tree structures
   * \author Martin Foerg
   * \date 2009-03-09 some comments (Thorsten Schindler)
   * \date 2009-03-19 element.h added (Thorsten Schindler)
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-07-17 implicit integration (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-06-20 revision of doxygen comments: add parameter names (Roland Zander)
    */
  class ObjectInterface {
    public:
      /**
       * \brief constructor
       */
      ObjectInterface() {}

      /**
       * \brief destructor
       */
      virtual ~ObjectInterface() {}

      /**
       * \brief update linear transformation \f$ \boldsymbol{T} \f$ between differentiated positions and generalised velocities
       * \param t simulation time
       */
      virtual void updateT(double t) = 0;

      /**
       * \brief update smooth right hand side \f[ \vh \f]
       * \param t simulation time
       */
      virtual void updateh(double t, int i=0) = 0;

      // TODO
      //virtual void updateh0Fromh1(double t) = 0;
      //virtual void updateud0Fromud1(double t) = 0;

      /**
       * \brief updates Jacobian for implicit integration regarding state
       * \param t simulation time
       */
      virtual void updatedhdz(double t) = 0;
      
      /**
       * \brief update mass matrix
       * \param t simulation time
       */
      virtual void updateM(double t, int i=0) = 0;

      /**
       * \brief update state dependent variables (e.g. kinematics of frames, contours and bodies)
       * \param t simulation time
       */
      virtual void updateStateDependentVariables(double t) = 0;

      /**
       * \brief update state derivative dependent variables (e.g. accelerations)
       * \param t simulation time
       */
      virtual void updateStateDerivativeDependentVariables(double t) = 0;

      /**
       * \brief update h-vector for inverse kinetics
       * \param t simulation time
       */
      virtual void updatehInverseKinetics(double t, int i=0) = 0;

      /**
       * \brief update acceleration description of frames, contours and bodies
       * \param t simulation time
       */
      virtual void updateJacobians(double t, int j=0) = 0;

      /**
       * \brief update position increment
       * \param  t simulation time
       * \param dt simulation time step size
       */
      virtual void updatedq(double t, double dt) = 0;

      /**
       * \brief update velocity increment
       * \param  t simulation time
       * \param dt simulation time step size
       */
      virtual void updatedu(double t, double dt) = 0;

      /**
       * \brief update differentiated velocity
       * \param t simulation time 
       */
      virtual void updateud(double t, int i=0) = 0;

      /**
       * \brief update differentiated positions
       * \param t simulation time
       */
      virtual void updateqd(double t) = 0;

      /**
       * \brief update differentiated state
       * \param t simulation time
       */
      virtual void updatezd(double t) = 0;

      /**
       * \param size of right hand side
       * \param index for normal usage and inverse kinetics 
       */
      virtual void sethSize(int hSize, int i=0) = 0;

      /**
       * \param index of right hand side
       * \param index for normal usage and inverse kinetics 
       */
      virtual void sethInd(int ind, int i=0) = 0;

      /**
       * \param index for normal usage and inverse kinetics 
       * \return size of right hand side
       */
      virtual int gethSize(int i=0) const = 0;

      /**
       * \return size of positions
       */
      virtual int getqSize() const = 0;

      /**
       * \param index for normal usage and inverse kinetics 
       * \return size of velocities
       */
      virtual int getuSize(int i=0) const = 0;

      /**
       * \brief calculates size of positions
       */
      virtual void calcqSize() = 0;

      /**
       * \brief calculates size of velocities
       * \param index for normal usage and inverse kinetics 
       */
      virtual void calcuSize(int j=0) = 0;

      /**
       * \return index of positions relative to dynamical system
       */
      virtual int getqInd(DynamicSystem* sys) = 0;

      /**
       * \param index for normal usage and inverse kinetics 
       * \return index of velocities relative to parent
       */
      virtual int getuInd(int i=0) = 0;

      /**
       * \param dynamic system
       * \param index for normal usage and inverse kinetics 
       * \return index of velocities relative to dynamical system
       */
      virtual int getuInd(DynamicSystem* sys, int i=0) = 0;

      /**
       * \param index of positions relative to parent
       */
      virtual void setqInd(int ind) = 0;

      /**
       * \param index of velocities relative to parent
       * \param index for normal usage and inverse kinetics 
       */
      virtual void setuInd(int ind, int i=0) = 0;

      /**
       * \param dynamic system
       * \param index for normal usage and inverse kinetics 
       * \return index of right hand side relative to dynamical system
       */
      virtual int gethInd(DynamicSystem* sys, int i=0) = 0;

      /**
       * \return positions
       */
      virtual const fmatvec::Vec& getq() const = 0; 
      
      /**
       * \return positions
       */
      virtual fmatvec::Vec& getq() = 0; 
      
      /**
       * \return velocities
       */
      virtual const fmatvec::Vec& getu() const = 0; 

      /**
       * \return velocities
       */
      virtual fmatvec::Vec& getu() = 0; 
      
      /**
       * \brief update JACOBIAN for inverse kinetics
       * \param t simulation time
       */
//      virtual void updateInverseKineticsJacobians(double t) = 0;

      /**
       * \return associated plot group
       */
      virtual H5::Group *getPlotGroup() = 0;

      /**
       * \return plot feature
       */
      virtual PlotFeatureStatus getPlotFeature(PlotFeature fp) = 0;

      /**
       * \return plot feature for derived classes
       */
      virtual PlotFeatureStatus getPlotFeatureForChildren(PlotFeature fp) = 0;

      /** 
       * Return the full path of the object.
       * \param pathDelim The delimiter of the path
       */
      virtual std::string getPath(char pathDelim='.') = 0;

#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() = 0;
#endif
  };

}

#endif /* _OBJECT_INTERFACE_H_ */


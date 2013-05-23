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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONSTRAINT_H
#define _CONSTRAINT_H

#include "object.h"
#include "utils/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Arrow;
}
#endif

namespace MBSim {

  class RigidBody;
  class Frame;

  /** 
   * \brief class for constraints between generalized coordinates of objects
   * \author Martin Foerg
   */
  class Constraint : public Object {
    private:

    public:
      Constraint(const std::string &name);
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() {return 0;}
#endif
  };

   /** 
   * \brief example 2 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class GearConstraint : public Constraint {

    public:
      GearConstraint(const std::string &name, RigidBody* body);
      GearConstraint(const std::string &name="");

      void addDependency(RigidBody* body_, double ratio);

      void init(InitStage stage);

      void setDependentBody(RigidBody* body_) {bd=body_; }

      void updateStateDependentVariables(double t);
      void updateJacobians(double t, int j=0);
      void setUpInverseKinetics();

      void initializeUsingXML(MBXMLUtils::TiXmlElement * element);

      virtual std::string getType() const { return "GearConstraint"; }
    
    private:
      std::vector<RigidBody*> bi;
      RigidBody *bd;
      std::vector<double> ratio;
      
      std::string saved_DependentBody;
      std::vector<std::string> saved_DependencyBodies;
      std::vector<double> saved_ratio;
  };

  /** 
   * \brief example 4 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class KinematicConstraint : public Constraint {

    public:
      KinematicConstraint(const std::string &name, RigidBody* body);
      KinematicConstraint(const std::string &name="");

      void setKinematicFunction(Function1<fmatvec::VecV,double>* f_) { f = f_;}
      void setFirstDerivativeOfKinematicFunction(Function1<fmatvec::VecV,double>* fd_) { fd = fd_;}
      void setSecondDerivativeOfKinematicFunction(Function1<fmatvec::VecV,double>* fdd_) { fdd = fdd_;}
      void setDependentBody(RigidBody* body) {bd=body; }
      virtual void setUpInverseKinetics();

      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updateJacobians(double t, int j=0);

      void initializeUsingXML(MBXMLUtils::TiXmlElement * element);

      virtual std::string getType() const { return "KinematicConstraint"; }

    private:
      RigidBody *bd;
      Function1<fmatvec::VecV,double> *f, *fd, *fdd;

      std::string saved_DependentBody;
  };

  /** 
   * \brief example 5 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   * 2011-08-04 XML Interface added (Markus Schneider)
   */
  class JointConstraint : public Constraint {
    public:
      JointConstraint(const std::string &name="");

      void init(InitStage stage);
      void initz();

      void connect(Frame* frame1, Frame* frame2);
      void setDependentBodiesFirstSide(std::vector<RigidBody*> bd);
      void setDependentBodiesSecondSide(std::vector<RigidBody*> bd);
      void setIndependentBody(RigidBody* bi);

      virtual void setUpInverseKinetics();
      void setForceDirection(const fmatvec::Mat3xV& d_) {dT = d_;}
      void setMomentDirection(const fmatvec::Mat3xV& d_) {dR = d_;}
      void setq0(const fmatvec::Vec& q0_) {q0 = q0_;}

      fmatvec::Vec res(const fmatvec::Vec& q, const double& t);
      void updateStateDependentVariables(double t); 
      void updateJacobians(double t, int j=0); 
      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      virtual std::string getType() const { return "JointConstraint"; }

      /** \brief Visualize a force arrow acting on frame2 */
      void setOpenMBVForceArrow(OpenMBV::Arrow *arrow) {
        FArrow = arrow;
      }

      /** \brief Visualize a moment arrow acting on frame2 */
      void setOpenMBVMomentArrow(OpenMBV::Arrow *arrow) {
        MArrow = arrow;
      }

    private:
      class Residuum : public Function1<fmatvec::Vec,fmatvec::Vec> {
        private:
          std::vector<RigidBody*> body1, body2;
          fmatvec::Mat3xV dT, dR;
          Frame *frame1, *frame2;
          double t;
          std::vector<Frame*> i1,i2;
        public:
          Residuum(std::vector<RigidBody*> body1_, std::vector<RigidBody*> body2_, const fmatvec::Mat3xV &dT_, const fmatvec::Mat3xV &dR_,Frame *frame1_, Frame *frame2_,double t_,std::vector<Frame*> i1_, std::vector<Frame*> i2_);
          fmatvec::Vec operator()(const fmatvec::Vec &x, const void * =NULL);
      };
      std::vector<RigidBody*> bd1;
      std::vector<RigidBody*> bd2;
      RigidBody *bi;
      std::vector<Frame*> if1;
      std::vector<Frame*> if2;

      Frame *frame1,*frame2;

      fmatvec::Mat3xV dT;
      fmatvec::Mat3xV dR;

      std::vector<fmatvec::Index> Iq1, Iq2, Iu1, Iu2, Ih1, Ih2;
      int nq, nu, nh;
      fmatvec::Vec q, u;
      fmatvec::Mat J;
      fmatvec::Vec j;
      fmatvec::Mat JT, JR;
      fmatvec::Vec q0;

      std::string saved_ref1, saved_ref2;
      std::vector<std::string> saved_RigidBodyFirstSide, saved_RigidBodySecondSide;
      std::string saved_IndependentBody;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Arrow *FArrow, *MArrow;
#endif
  };

}

#endif

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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _CONSTRAINT_H
#define _CONSTRAINT_H

#include "object.h"
#include "utils/function.h"

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
      void updateInverseKineticsJacobians(double t) {}
#ifdef HAVE_OPENMBVCPPINTERFACE
      virtual OpenMBV::Group* getOpenMBVGrp() {return 0;}
#endif
  };

   /** 
   * \brief example 2 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class Constraint2 : public Constraint {
    private:
      std::vector<RigidBody*> bi;
      RigidBody *bd;
      std::vector<double> ratio;

//      std::vector<Function2<fmatvec::Vec, fmatvec::Vec, double>*> fd;
//      std::vector<Function2<fmatvec::Mat, fmatvec::Vec, double>*> fdJ;
//      std::vector<Function1<fmatvec::Vec,double>*> fdj;
//      std::vector<Function3<fmatvec::Mat, fmatvec::Vec, fmatvec::Vec, double>*> fdJd;
//      std::vector<Function1<fmatvec::Vec,double>*> fdjd;
    public:
      Constraint2(const std::string &name, RigidBody* body);

      void addDependency(RigidBody* body_, double ratio);

      void init(InitStage stage);

      void updateStateDependentVariables(double t);
      void updateJacobians(double t);
  };

  /** 
   * \brief example 3 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class Constraint3 : public Constraint {
    private:
      RigidBody *bd;

    public:
      Constraint3(const std::string &name, RigidBody* body);

      void updateStateDependentVariables(double t);
      void updateJacobians(double t);
  };

  /** 
   * \brief example 5 for contraint 
   * \todo generalization of this class
   * \author Martin Foerg
   */
  class JointConstraint : public Constraint {
    protected:
      class Residuum : public Function1<fmatvec::Vec,fmatvec::Vec> {
	std::vector<RigidBody*> body1, body2;
	fmatvec::Mat dT, dR;
	Frame *frame1, *frame2;
	double t;
	std::vector<int> i1,i2;
	public:
	Residuum(std::vector<RigidBody*> body1_, std::vector<RigidBody*> body2_, const fmatvec::Mat &dT_, const fmatvec::Mat &dR_,Frame *frame1_, Frame *frame2_,double t_,std::vector<int> i1_, std::vector<int> i2_);
	fmatvec::Vec operator()(const fmatvec::Vec &x, const void * =NULL);
      };
      std::vector<RigidBody*> bd1;
      std::vector<RigidBody*> bd2;
      RigidBody *bi;
      std::vector<int> if1;
      std::vector<int> if2;

      Frame *frame1,*frame2;

      fmatvec::Mat dT;
      fmatvec::Mat dR;

      std::vector<fmatvec::Index> Iq1, Iq2, Iu1, Iu2, Ih1, Ih2;
      int nq, nu, nh;
      fmatvec::Vec q, u;
      fmatvec::Mat J;
      fmatvec::Vec j;
      fmatvec::Mat JT, JR;
      fmatvec::Vec q0;

    public:
      JointConstraint(const std::string &name);

      void init(InitStage stage);
      void initz();

      void connect(Frame* frame1, Frame* frame2);
      void setDependentBodiesFirstSide(std::vector<RigidBody*> bd);
      void setDependentBodiesSecondSide(std::vector<RigidBody*> bd);
      void setIndependentBody(RigidBody* bi);

      void setForceDirection(const fmatvec::Mat& d_) {dT = d_;}
      void setMomentDirection(const fmatvec::Mat& d_) {dR = d_;}
      void setq0(const fmatvec::Vec& q0_) {q0 = q0_;}

      fmatvec::Vec res(const fmatvec::Vec& q, const double& t);
      void updateStateDependentVariables(double t); 
      void updateJacobians(double t); 
      virtual void initializeUsingXML(TiXmlElement *element);

    private:
      std::string saved_ref1, saved_ref2;
  };

}

#endif

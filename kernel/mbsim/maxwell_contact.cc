#include <config.h>

#include <maxwell_contact.h>

#include <fmatvec.h>

#include <mbsim/contour.h>
#include <mbsim/contour_pdata.h>
#include <mbsim/contact_kinematics/contact_kinematics.h>
#include <mbsim/constitutive_laws.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/contact_utils.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/nonsmooth_algebra.h>
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/utils/eps.h>
#include <mbsim/object_interface.h>
#include <mbsim/utils/rotarymatrices.h>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/frame.h>
#include <openmbvcppinterface/arrow.h>
#include <openmbvcppinterface/objectfactory.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  /* MaxwellContact   ***************************************************************************************************************************/

  MaxwellContact::MaxwellContact(const string &name) :
      LinkMechanics(name), contactKinematics(0), fdf(0), cpData(0), gActive(0), gActive0(0), gk(0), gdk(0), lak(0), WF(0), Vk(0), Wk(0), laSizek(0), laIndk(0), gSizek(0), gIndk(0), gdSizek(0), gdIndk(0), svSizek(0), svIndk(0), rFactorSizek(0), rFactorIndk(0), possibleContactPoints(0), plotContactPoints(0), matConst(1.), matConstSetted(false), gTol(1.), INFO(false)
#ifdef HAVE_OPENMBVCPPINTERFACE
          , openMBVContactGrp(0), openMBVContactFrame(0), openMBVNormalForceArrow(0), openMBVFrictionForceArrow(0), openMBVContactFrameSize(0), openMBVContactFrameEnabled(true), normalForceArrow(false), frictionForceArrow(false)
#endif
  {
  }

  MaxwellContact::~MaxwellContact() {
    for (vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
      delete[] *i;
    for (vector<ContactKinematics *>::iterator i = contactKinematics.begin(); i != contactKinematics.end(); ++i)
      delete *i;
    for (vector<Vec*>::iterator i = WF.begin(); i != WF.end(); ++i)
      delete[] *i;
  }

  /**
   * \brief solves the LCP to compute contact forces as well as distances
   */
  void MaxwellContact::updateh(double t) {

    updatePossibleContactPoints();

    if (possibleContactPoints.size() > 0) {

      /*compute Influence Matrix*/
      updateC(t);

      computeMaterialConstant(t);

      /*save rigidBodyGaps in vector*/
      Vec rigidBodyGap(possibleContactPoints.size(), INIT, 0.);
      for (size_t i = 0; i < possibleContactPoints.size(); i++) {
        rigidBodyGap(i) = gk[possibleContactPoints[i]](0);
      }

      if (INFO)
        cout << "rigidBodyGap: " << rigidBodyGap << endl;

      /*create the Maxwell Function*/

      MaxwellFunction func = MaxwellFunction(rigidBodyGap, C);

      /*Initialize gapLamba*/
      Vec gapLambda0(rigidBodyGap.size() * 2, INIT, 0.);
      for (int i = 0; i < rigidBodyGap.size(); i++) {
        if (rigidBodyGap(i) > 0)
          gapLambda0(i) = rigidBodyGap(i);
        else
          gapLambda0(i) = 0;
      }
      for (int i = rigidBodyGap.size(); i < rigidBodyGap.size() * 2; i++) {
        if (rigidBodyGap(i - rigidBodyGap.size()) > 0)
          gapLambda0(i) = 0;
        else
          gapLambda0(i) = -rigidBodyGap(i - rigidBodyGap.size()) / matConst;
      }

      if (INFO)
        cout << "gapLambda0: " << gapLambda0 << endl;

      //TODO_grundl build Jacobian, if it is possible at all --> if it is computed numerically it gets more unstable?
      MaxwellJacobian jac;

      /* solve the LCP */

      //Use Newton Solver
      MultiDimNewtonMethod NewtonSolver(&func);
      MultiDimFixPointIteration FixpointIterator(&func);

      //NOTE: Fortran-NewtonSolver seems to work worse than the "native" NewtonSolver of the utils in mbsim
      //      MultiDimNewtonMethodFortran NewtonSolverFortran;
      //      NewtonSolverFortran.setRootFunction(&func);
      //      Vec gapLambdaFortran = NewtonSolverFortran.solve(gapLambda0);

      bool converged = false;
      int loop = 0;
      int maxloops = 2;

      //use Lemke-Solver for small systems
      if(possibleContactPoints.size() < 10) //TODO: add paramter for this option
        maxloops = 0;

      Vec gapLambda(gapLambda0.copy());

      Vec lambda;
      lambda >> gapLambda(rigidBodyGap.size(), rigidBodyGap.size() * 2 - 1);

      while (loop++ < maxloops and converged == false) {
        if (loop == maxloops)
          NewtonSolver.setTolerance(1.e-6);
        func.setSolverType(NewtonMethodSolver);
        gapLambda = NewtonSolver.solve(gapLambda);  //use gap Lambda of Fixpoint Solver as it probably is a better starting value

        if (INFO) { //Newton-Sovler Info
          cout << "Timestep: t=" << t << endl;
          cout << "Info about NewtonSolver" << endl;
          cout << "gapLambda0: " << gapLambda0 << endl;
          cout << "gapLambda: " << gapLambda << endl;
          cout << "nrm2(f(gapLambda)): " << nrm2(func(gapLambda)) << endl;
          cout << "nrm2(f(gapLambda0)): " << nrm2(func(gapLambda0)) << endl;
        }

        switch (NewtonSolver.getInfo()) {
          case 0:
            converged = true;
          break;
          case 1: //convergence is active, but not enough steps
            if (INFO) {
              cout << "MaxwellContact::updateh(double t): Newton scheme seems to converge but has'nt finished in loop No:" << loop << endl;
              cout << "Increasing number of maximal iterations ... " << endl;
            }
            NewtonSolver.setMaximumNumberOfIterations(NewtonSolver.getNumberOfMaximalIterations() * 10);  //raise number of iterations (if there is convergence ...)
            if (maxloops == loop)
              maxloops++;
          break;
          case -1:
            if (INFO) {
              cout << "MaxwellContact::updateh(double t): No convergence of Newton scheme during calculation of contact forces in loop No:" << loop << endl;
              cout << "Trying a fixpoint-solver (at least for a good starting value for the newton scheme) ..." << endl;
            }

            if(loop < maxloops) {

              func.setSolverType(FixPointIterationSolver);
              gapLambda = FixpointIterator.solve(gapLambda);

              if (INFO) {
                cout << "Timestep: t=" << t << endl;
                cout << "Info about FixpointSolver" << endl;
                cout << "gapLambda0: " << gapLambda0 << endl;
                cout << "gapLambda: " << gapLambda << endl;
                func.setSolverType(NewtonMethodSolver);
                cout << "f(gapLambda): " << nrm2(func(gapLambda)) << endl;
                cout << "f(gapLambda0): " << nrm2(func(gapLambda0)) << endl;
              }

              if (FixpointIterator.getInfo() == 0) {
                if (INFO)
                  cout << "Maxwell Contact converged with Fixpoint-Iterator in loop " << loop << endl;
                converged = true;
              }
            }

          break;
          default:
            throw MBSimError("ERROR MaxwellContact::updateh(double t): No convergence during calculation of contact forces with Newton scheme!");
          break;
        }
      }

      if (!converged) {
        if (INFO) {
          cout << "WARNING MaxwellContact::updateh(double t): No convergence during calculation of contact forces with Newton scheme!" << endl;
          cout << "                                           Now using Lemke Algorithm..." << endl;
        }

        SqrMat C_(C.size(),NONINIT);
        for(int i=0; i < C.size(); i++)
          for(int j=0; j < C.size(); j++)
            C_(i,j) = C(i,j);

        LemkeAlgorithm Lemke(C_, rigidBodyGap);
        gapLambda = Lemke.solve();
      }
      else {
        if (INFO) {
          cout << "The result of the NewtonSolver (steps=" << NewtonSolver.getNorms().size() << ") is: " << gapLambda << endl;
          cout << "with lambda: " << lambda << endl;
        }
      }

      //index for active contact (to assign possible contact to active contact)
      int activeContact = 0;

      /*create h - vector(s)*/
      //loop over all contacts
      for (size_t potentialContact = 0; potentialContact < contactKinematics.size(); potentialContact++) {
        if (gActive[potentialContact]) { //if contact is active
          /*compute forces with directions for contact*/
          lak[potentialContact](0) = lambda(activeContact); //normal force
          WF[potentialContact][1] = cpData[potentialContact][0].getFrameOfReference().getOrientation().col(0) * lak[potentialContact](0);
          if (fdf) { //if friction is activated
            int frictionDirections = fdf->getFrictionDirections();
            lak[potentialContact](1, frictionDirections) = (*fdf)(gdk[potentialContact](1, frictionDirections), fabs(lak[potentialContact](0)));

            WF[potentialContact][1] += cpData[potentialContact][0].getFrameOfReference().getOrientation().col(1) * lak[potentialContact](1);
            if (frictionDirections > 1)
              WF[potentialContact][1] += cpData[potentialContact][0].getFrameOfReference().getOrientation().col(2) * lak[potentialContact](2);
          }
          WF[potentialContact][0] = -WF[potentialContact][1];
          for (size_t i = 0; i < 2; i++) { //add forces to h-vector of both contours of the current contact point
            h[2 * potentialContact + i] += cpData[potentialContact][i].getFrameOfReference().getJacobianOfTranslation().T() * WF[potentialContact][i];
            hLink[2 * potentialContact + i] += cpData[potentialContact][i].getFrameOfReference().getJacobianOfTranslation().T() * WF[potentialContact][i];
          }

          //increase for next active contacts
          activeContact++;
        }
      }
    }
  }

  /**
   * \brief updates the rigid-body distances for all contact-points
   */
  void MaxwellContact::updateg(double t) {
    for (size_t i = 0; i < contactKinematics.size(); i++) {
      contactKinematics[i]->updateg(gk[i], cpData[i]);
    }
  }

  /**
   * \brief updates the velocities on active contact points
   */
  void MaxwellContact::updategd(double t) {
    for (size_t k = 0; k < contactKinematics.size(); k++) {
      if (gActive[k]) {
        //update velocities on contour-points
        for (int i = 0; i < 2; i++)
          contour[2 * k + i]->updateKinematicsForFrame(cpData[k][i], velocities); // angular velocity necessary e.g. see ContactKinematicsSpherePlane::updatewb

        Vec Wn = cpData[k][0].getFrameOfReference().getOrientation().col(0);

        Vec WvD = cpData[k][1].getFrameOfReference().getVelocity() - cpData[k][0].getFrameOfReference().getVelocity();

        //write normal-velocity in vector
        gdk[k](0) = Wn.T() * WvD;

        if (gdk[k].size() > 1) { //are there more velocity-directions needed?
          Mat Wt(3, gdk[k].size() - 1);
          Wt.col(0) = cpData[k][0].getFrameOfReference().getOrientation().col(1);
          if (gdk[k].size() > 2)
            Wt.col(1) = cpData[k][0].getFrameOfReference().getOrientation().col(2);

          //write second (and third for 3D-contact) velocity into vector
          gdk[k](1, gdk[k].size() - 1) = Wt.T() * WvD;
        }
      }
    }
  }

  void MaxwellContact::updateJacobians(double t) {
    for (size_t k = 0; k < contactKinematics.size(); k++)
      if (gActive[k])
        for (int i = 0; i < 2; i++)
          contour[2 * k + i]->updateJacobiansForFrame(cpData[k][i]);
  }

  /**
   * \brief references the h and hLink vectors of the single contour-pairings to the global vector
   */
  void MaxwellContact::updatehRef(const Vec& hParent, const Vec& hLinkParent, int j) {
    for (size_t i = 0; i < contour.size(); i++) {
      int hInd = contour[i]->getParent()->gethInd(parent, j);
      Index I = Index(hInd, hInd + contour[i]->gethSize(j) - 1);
      h[i].resize() >> hParent(I);
      hLink[i].resize() >> hLinkParent(I);
    }
  }

  void MaxwellContact::init(InitStage stage) {
    //    LinkMechanics::init(stage); //TODO: Ist Reihenfolge wichtig, wenn ja, wo ist das dokumentiert --> Kommentar in extradynamic-interface !
    if (stage == MBSim::preInit) {
      //Create Vector for every possible contact point
      for (size_t k = 0; k < contactKinematics.size(); k++) {
        gk.push_back(Vec());
        gdk.push_back(Vec());
        lak.push_back(Vec());
      }
    }
    else if (stage == resize) {
      int maxNumberOfContacts = contactKinematics.size();

      g.resize(maxNumberOfContacts);
      gActive.resize(maxNumberOfContacts);
      gd.resize(maxNumberOfContacts * (1 + getFrictionDirections()));
      la.resize(maxNumberOfContacts * (1 + getFrictionDirections()));

      for (vector<ContourPointData*>::iterator i = cpData.begin(); i != cpData.end(); ++i)
        delete[] *i; //the cpData has to be cleared, as the resize-stage is executed twice at the initialisation
      cpData.clear();
      for (int i = 0; i < maxNumberOfContacts; i++) {
        cpData.push_back(new ContourPointData[2]);
        //TODO: seems to be more complicated as it has to be ...
        cpData[i][0].getFrameOfReference().setName("0");
        cpData[i][1].getFrameOfReference().setName("1");
        cpData[i][0].getFrameOfReference().getJacobianOfTranslation().resize();
        cpData[i][0].getFrameOfReference().getJacobianOfRotation().resize();
        cpData[i][1].getFrameOfReference().getJacobianOfTranslation().resize();
        cpData[i][1].getFrameOfReference().getJacobianOfRotation().resize();

        cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(0), 0);
        cpData[i][0].getFrameOfReference().sethSize(contour[0]->gethSize(1), 1);
        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(0), 0);
        cpData[i][1].getFrameOfReference().sethSize(contour[1]->gethSize(1), 1);

        WF.push_back(new Vec[2]);
        WF[i][0].resize(3);
        WF[i][1].resize(3);
      }

    }
    else if (stage == MBSim::plot) {
      updatePlotFeatures(parent);
      if (getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || normalForceArrow || frictionForceArrow)) {
          openMBVContactGrp = new OpenMBV::Group();
          openMBVContactGrp->setName(name + "_ContactGroup");
          openMBVContactGrp->setExpand(false);
          parent->getOpenMBVGrp()->addObject(openMBVContactGrp);

          for (size_t iterPlotContactPoint = 0; iterPlotContactPoint < plotContactPoints.size(); iterPlotContactPoint++) {
            if (openMBVContactFrameSize > epsroot()) {
              openMBVContactFrame.push_back(vector<OpenMBV::Frame*>(2, new OpenMBV::Frame));
              for (unsigned int k = 0; k < 2; k++) { // frames
                openMBVContactFrame[iterPlotContactPoint][k]->setOffset(1.);
                openMBVContactFrame[iterPlotContactPoint][k]->setSize(openMBVContactFrameSize);
                openMBVContactFrame[iterPlotContactPoint][k]->setName("ContactPoint_" + numtostr((int) plotContactPoints[iterPlotContactPoint]) + (k == 0 ? "A" : "B"));
                openMBVContactFrame[iterPlotContactPoint][k]->setEnable(openMBVContactFrameEnabled);
                openMBVContactGrp->addObject(openMBVContactFrame[iterPlotContactPoint][k]);
              }
              // arrows
              OpenMBV::Arrow *arrow;
              if (normalForceArrow) {
                arrow = new OpenMBV::Arrow(*normalForceArrow);
                arrow->setName("NormalForce_" + numtostr((int) plotContactPoints[iterPlotContactPoint]));
                openMBVNormalForceArrow.push_back(arrow); // normal force
                openMBVContactGrp->addObject(arrow);
              }

              if (frictionForceArrow && getFrictionDirections() > 0) { // friction force
                arrow = new OpenMBV::Arrow(*frictionForceArrow);
                arrow->setName("FrictionForce_" + numtostr((int) plotContactPoints[iterPlotContactPoint]));
                openMBVFrictionForceArrow.push_back(arrow);
                openMBVContactGrp->addObject(arrow);
              }
            }
          }
        }
#endif
        for (size_t iterPlotContactPoint = 0; iterPlotContactPoint < plotContactPoints.size(); iterPlotContactPoint++) {
          if (getPlotFeature(generalizedLinkForce) == enabled) {
            for (int j = 0; j < 1 + getFrictionDirections(); ++j)
              plotColumns.push_back("la[" + numtostr((int) plotContactPoints[iterPlotContactPoint]) + "](" + numtostr(j) + ")");
          }
          if (getPlotFeature(linkKinematics) == enabled) {
            plotColumns.push_back("g[" + numtostr((int) plotContactPoints[iterPlotContactPoint]) + "](" + numtostr(0) + ")");
            for (int j = 0; j < 1 + getFrictionDirections(); ++j)
              plotColumns.push_back("gd[" + numtostr((int) plotContactPoints[iterPlotContactPoint]) + "](" + numtostr(j) + ")");
          }
        }
      }
    }
    else if (stage == MBSim::unknownStage) {

      // reference to the positions of (full) vectors of link-class
      for (size_t k = 0; k < contactKinematics.size(); k++) {
        gk[k].resize() >> g(k, k);
        gdk[k].resize() >> gd(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
        lak[k].resize() >> la(k * (1 + getFrictionDirections()), (k + 1) * (1 + getFrictionDirections()) - 1);
      }
    }
    //As last step: do init-step of LinkMechanics
    LinkMechanics::init(stage);
  }

  void MaxwellContact::checkActiveg() {
    for (unsigned i = 0; i < contactKinematics.size(); i++) {
      gActive[i] = gk[i](0) < gTol ? true : false;
    }
  }

  void MaxwellContact::resizeJacobians(int j) {
    checkActiveg(); //TODO do it at another place!!!
    for (size_t k = 0; k < contactKinematics.size(); k++) {
      if (gActive[k]) {
        for (size_t i = 0; i < 2; i++)
          cpData[k][i].getFrameOfReference().resizeJacobians(j);
      }
    }
  }

  /**
   * \brief plot-routine for output
   */
  void MaxwellContact::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
      for (size_t iterPlotContactPoint = 0; iterPlotContactPoint < plotContactPoints.size(); iterPlotContactPoint++) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if (getPlotFeature(openMBV) == enabled && (openMBVContactFrameSize > epsroot() || normalForceArrow || frictionForceArrow)) {
          // frames
          if (openMBVContactFrameSize > epsroot()) {
            for (int k = 0; k < 2; k++) {
              vector<double> data;
              data.push_back(t);
              data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][k].getFrameOfReference().getPosition()(0));
              data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][k].getFrameOfReference().getPosition()(1));
              data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][k].getFrameOfReference().getPosition()(2));
              Vec cardan = AIK2Cardan(cpData[plotContactPoints[iterPlotContactPoint]][k].getFrameOfReference().getOrientation());
              data.push_back(cardan(0));
              data.push_back(cardan(1));
              data.push_back(cardan(2));
              data.push_back(0);
              openMBVContactFrame[iterPlotContactPoint][k]->append(data);
            }
          }

          // arrows
          // normal force
          vector<double> data;
          if (normalForceArrow) {
            data.push_back(t);
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(2));
            Vec F(3, INIT, 0);
            F = cpData[plotContactPoints[iterPlotContactPoint]][0].getFrameOfReference().getOrientation().col(0) * lak[plotContactPoints[iterPlotContactPoint]](0);
            data.push_back(F(0));
            data.push_back(F(1));
            data.push_back(F(2));
            data.push_back(nrm2(F));
            openMBVNormalForceArrow[iterPlotContactPoint]->append(data);
          }
          if (frictionForceArrow && getFrictionDirections() > 0) { // friction force
            data.clear();
            data.push_back(t);
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(0));
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(1));
            data.push_back(cpData[plotContactPoints[iterPlotContactPoint]][1].getFrameOfReference().getPosition()(2));
            Vec F(3, INIT, 0);
            F = cpData[plotContactPoints[iterPlotContactPoint]][0].getFrameOfReference().getOrientation().col(1) * lak[plotContactPoints[iterPlotContactPoint]](1);
            if (getFrictionDirections() > 1)
              F += cpData[plotContactPoints[iterPlotContactPoint]][0].getFrameOfReference().getOrientation().col(2) * lak[plotContactPoints[iterPlotContactPoint]](2);
            data.push_back(F(0));
            data.push_back(F(1));
            data.push_back(F(2));
            data.push_back((isSetValued() && lak[plotContactPoints[iterPlotContactPoint]].size() > 1) ? 1:0.5); // draw in green if slipping and draw in red if sticking
            openMBVFrictionForceArrow
            [iterPlotContactPoint]->append(data);
          }
          //        }
        }
#endif
        if (getPlotFeature(generalizedLinkForce) == enabled) {
          if (gActive[plotContactPoints[iterPlotContactPoint]]) {
            plotVector.push_back(lak[plotContactPoints[iterPlotContactPoint]](0));
            if (fdf) {
              for (int j = 1; j < 1 + getFrictionDirections(); j++)
                plotVector.push_back(lak[plotContactPoints[iterPlotContactPoint]](j));
            }
          }
          else {
            for (int j = 0; j < 1 + getFrictionDirections(); j++)
              plotVector.push_back(0);
          }
        }
        if (getPlotFeature(linkKinematics) == enabled) {
          plotVector.push_back(gk[plotContactPoints[iterPlotContactPoint]](0)); //gN
          for (int j = 0; j < 1 + getFrictionDirections(); j++)
            plotVector.push_back(gdk[plotContactPoints[iterPlotContactPoint]](j)); //gd
        }
      }
    }
    LinkMechanics::plot(t, dt);
  }

  void MaxwellContact::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      LinkMechanics::closePlot();
    }
  }

  void MaxwellContact::setPlotContactPoint(const int & contactNumber, bool enable) {
    assert(contactNumber >= 0);
    assert(contactNumber < (int)contactKinematics.size());

    if (!enable) { //delete item, if it is in List already ...
      for (std::vector<int>::iterator iter = plotContactPoints.begin(); iter != plotContactPoints.end(); ++iter) {
        if (*iter == contactNumber) {
          plotContactPoints.erase(iter);
          return;
        }
      }
    }
    else { //enable plot of contactNumber
      for (std::vector<int>::iterator iter = plotContactPoints.begin(); iter != plotContactPoints.end(); iter++) {
        if (*iter == contactNumber) {
          return; //if contactNumber is already in List --> return out of the function
        }
      }
      //append contactNumber to list, if it is not in the list
      plotContactPoints.push_back(contactNumber);
    }
  }

  int MaxwellContact::getFrictionDirections() {
    if (fdf)
      return fdf->getFrictionDirections();
    else
      return 0;
  }

  void MaxwellContact::add(Contour *contour1, Contour *contour2, bool plotContact /*= true*/, ContactKinematics * contactKinematics_ /* = 0*/) {
    if (contactKinematics_ == 0)
      contactKinematics_ = contour1->findContactPairingWith(contour1->getType(), contour2->getType());
    if (contactKinematics_ == 0)
      contactKinematics_ = contour1->findContactPairingWith(contour2->getType(), contour1->getType());
    if (contactKinematics_ == 0)
      contactKinematics_ = contour2->findContactPairingWith(contour2->getType(), contour1->getType());
    if (contactKinematics_ == 0)
      contactKinematics_ = contour2->findContactPairingWith(contour1->getType(), contour2->getType());
    if (contactKinematics_ == 0)
      throw MBSimError("Unknown contact pairing between Contour \"" + contour1->getType() + "\" and Contour\"" + contour2->getType() + "\"!");

    //append contactKinematics to vector
    contactKinematics_->assignContours(contour1, contour2);
    contactKinematics.push_back(contactKinematics_);

    if (plotContact)
      plotContactPoints.push_back(contactKinematics.size() - 1);

    //append contours 1 and 2 to list
    //TODO: optimization may be possible by adding one contour just once --> Problem: referencing to the contour gets harder
    LinkMechanics::connect(contour1);
    LinkMechanics::connect(contour2);
  }

  void MaxwellContact::addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct) {
    pair<Contour*, Contour*> Pair(contour1, contour2);
    if (contour2 < contour1)
      Pair = pair<Contour*, Contour*>(contour2, contour1);
    if (!influenceFunctions.count(Pair)) {
      influenceFunctions[Pair] = fct;
    }
    else {
      cout << "WARNING: Function existed for contour-pair: \"" << contour1->getName() << "\" + \"" << contour2->getName() << "\"." << endl;
      cout << "         No Function has been added." << endl;
    }

  }

  void MaxwellContact::updatePossibleContactPoints() {
    checkActiveg(); //TODO_grundl maybe is another location more reasonable. checkActiveg is normally called "global" in the dynamic_system_solver-class for all SetValued-Force laws. This is not ideal for the MaxwellContact so it calls this function on its own. why shouldn't call each link its own update-routine and the dynamic-system-solve is not forced to do it?
    possibleContactPoints.clear();
    for (size_t i = 0; i < gActive.size(); i++)
      if (gActive[i])
        possibleContactPoints.push_back(i);
  }

  void MaxwellContact::updateC(const double t) {
    C.resize(possibleContactPoints.size());

    for (size_t i = 0; i < possibleContactPoints.size(); i++) {
      //get contours of current possible contactPoint
      int currentContactNumber = possibleContactPoints[i];

      C(i, i) = computeFactorC(currentContactNumber);

      for (size_t j = i + 1; j < possibleContactPoints.size(); j++) {
        //get coupled contours
        int coupledContactNumber = possibleContactPoints[j];

        /*coupling of first current contour*/
        C(i, j) = computeFactorC(currentContactNumber, coupledContactNumber);
      }
    }

    if (INFO) {
      cout << "The InfluenceMatrix is: " << C << endl;
      cout << "With eigenvalues: " << eigval(C) << endl;
    }
  }

  double MaxwellContact::computeFactorC(const int &currentContactNumber) {
    double FactorC = 0.;

    int currentContourNumber = 2 * currentContactNumber;

    for (int i = 0; i < 2; i++) {

      //get involved contours
      Contour *contour1 = contour[currentContourNumber + i];
      Contour *contour2 = contour[currentContourNumber + i];
      pair<Contour*, Contour*> Pair;

      if (contour1 < contour2)
        Pair = pair<Contour*, Contour*>(contour1, contour2);
      else
        Pair = pair<Contour*, Contour*>(contour2, contour1);

      if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
        InfluenceFunction *fct = influenceFunctions[Pair];
        /*TODO_grundl: solve the problem of the ordering input parameters for the function (for now it isn't really solved ..)
         maybe add a MaxwellCouplingFunction class that is derived from Function2 but has these identifiers --> Identifiers are now the (short)names*/
        Vec firstLagrangeParameter;
        Vec secondLagrangeParameter;
        if (contour1->getShortName() == (*fct).getFirstContourName() and contour2->getShortName() == (*fct).getSecondContourName()) {
          firstLagrangeParameter.resize() = contour1->computeLagrangeParameter(cpData[currentContactNumber][i].getFrameOfReference().getPosition());
          secondLagrangeParameter.resize() = contour2->computeLagrangeParameter(cpData[currentContactNumber][i].getFrameOfReference().getPosition());
        }
        else
          throw MBSimError("MaxwellContact::computeFactorC: The contours \"" + contour1->getShortName() + "\" and \"" + contour2->getShortName() + " don't fit with the function's contour names: \"" + (*fct).getFirstContourName() + "\" and \"" + (*fct).getSecondContourName() + "\"");

        if (INFO) {
          cout << "First LagrangeParameter of contour \"" << contour1->getShortName() << "\" is:" << firstLagrangeParameter << endl;
          cout << "Second LagrangeParameter contour \"" << contour2->getShortName() << "\" is:" << secondLagrangeParameter << endl;
        }

        FactorC += (*fct)(firstLagrangeParameter, secondLagrangeParameter);
      }
    }

    if (fabs(FactorC) <= epsroot()) {
      throw MBSimError("No elasticity is given for one of the following contours:\n  -" + contour[currentContourNumber]->getShortName() + "\n  -" + contour[currentContourNumber + 1]->getShortName() + "\nThat is not an option!");
    }

    return FactorC;
  }

  double MaxwellContact::computeFactorC(const int &affectedContactNumber, const int &coupledContactNumber) {
    double FactorC = 0;

    int affectedContourNumber = 2 * affectedContactNumber;
    int coupledContourNumber = 2 * coupledContactNumber;

    for (int affectedContourIterator = 0; affectedContourIterator < 2; affectedContourIterator++) {
      for (int coupledContourIterator = 0; coupledContourIterator < 2; coupledContourIterator++) {
        //get involved contours
        Contour *contour1 = contour[affectedContourNumber + affectedContourIterator];
        Contour *contour2 = contour[coupledContourNumber + coupledContourIterator];

        pair<Contour*, Contour*> Pair;

        if (contour1 < contour2)
          Pair = pair<Contour*, Contour*>(contour1, contour2);
        else
          Pair = pair<Contour*, Contour*>(contour2, contour1);

        if (influenceFunctions.count(Pair)) { //If there is a function, there is a coupling between these contours
          InfluenceFunction *fct = influenceFunctions[Pair];
          /*TODO_grundl: solve the problem of the ordering for the function input parameters (for now it isn't really solved ..)
           maybe add a MaxwellCouplingFunction class that is derived from Function2 but has these identifiers*/
          Vec firstLagrangeParameter = Vec(2, NONINIT);
          Vec secondLagrangeParameter = Vec(2, NONINIT);
          if (contour1->getShortName() == (*fct).getFirstContourName() and contour2->getShortName() == (*fct).getSecondContourName()) {
            firstLagrangeParameter = contour1->computeLagrangeParameter(cpData[affectedContactNumber][affectedContourIterator].getFrameOfReference().getPosition());
            secondLagrangeParameter = contour2->computeLagrangeParameter(cpData[coupledContactNumber][coupledContourIterator].getFrameOfReference().getPosition());
          }
          else if (contour2->getShortName() == (*fct).getFirstContourName() and contour1->getShortName() == (*fct).getSecondContourName()) {
            secondLagrangeParameter = contour1->computeLagrangeParameter(cpData[affectedContactNumber][affectedContourIterator].getFrameOfReference().getPosition());
            firstLagrangeParameter = contour2->computeLagrangeParameter(cpData[coupledContactNumber][coupledContourIterator].getFrameOfReference().getPosition());
          }
          else
            throw MBSimError("MaxwellContact::computeFactorC: The contours \"" + contour1->getShortName() + "\" and \"" + contour2->getShortName() + " don't fit with the function's contour names: \"" + (*fct).getFirstContourName() + "\" and \"" + (*fct).getSecondContourName() + "\"");

          if (INFO) {
            cout << "First LagrangeParameter of contour \"" << contour1->getShortName() << "\" is:" << firstLagrangeParameter << endl;
            cout << "Second LagrangeParameter contour \"" << contour2->getShortName() << "\" is:" << secondLagrangeParameter << endl;
          }

          FactorC += (*fct)(firstLagrangeParameter, secondLagrangeParameter);
        }
      }
    }
    return FactorC;
  }

  void MaxwellContact::computeMaterialConstant(const double & t) {
    if (!matConstSetted && t > 0.) {
      /*update Material constant*/
      Vec Eigvals = eigval(C);
      double eigvalSum = 0;
      for (int i = 0; i < Eigvals.size(); i++) {
        eigvalSum += Eigvals(i);
      }
      matConst = eigvalSum / Eigvals.size();

      matConstSetted = true;
    }
  }

  /* END - MaxwellContact   ***************************************************************************************************************************/

  /* MaxwellFunction   ***************************************************************************************************************************/

  MaxwellFunction::MaxwellFunction(const fmatvec::Vec & rigidBodyGap_, const SymMat &C_, const double &r_ /*= 10*/, bool INFO_ /*= false*/) :
      NumberOfContacts(rigidBodyGap_.size()), rigidBodyGap(rigidBodyGap_), C(C_), r(r_), solverType(NewtonMethodSolver), INFO(INFO_) {

    //dimensions have to be equal
    assert(C_.size() == NumberOfContacts);
  }

  MaxwellFunction::~MaxwellFunction() {

  }

  Vec MaxwellFunction::operator ()(const Vec &gapLambda, const void *) {

    //check dimensions
    assert(gapLambda.size() == 2 * NumberOfContacts);

    Vec returnVec(2 * NumberOfContacts, INIT, 0.);

    //reference to gap and lambda
    Vec gap;
    Vec lambda;
    gap << gapLambda(0, NumberOfContacts - 1);
    lambda << gapLambda(NumberOfContacts, 2 * NumberOfContacts - 1);

    if (INFO) {

      cout << "gap is: " << gap << endl;
      cout << "lambda is: " << lambda << endl;

      cout << "C is: " << C << endl;
    }

    //compute first part
    if (solverType == NewtonMethodSolver) {
      returnVec(0, NumberOfContacts - 1) = rigidBodyGap + C * lambda - gap;

      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(lambda(contactIterator) - r * gap(contactIterator)) - lambda(contactIterator);
        if (INFO) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(lambda(contactIterator) - r * gap(contactIterator)) << "- " << lambda(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << lambda(contactIterator) << "-" << r << "*" << gap(contactIterator) << endl;
        }
      }
    }
    else if (solverType == FixPointIterationSolver) {
      returnVec(0, NumberOfContacts - 1) = rigidBodyGap + C * lambda;
      //loop for the prox-functions
      for (int contactIterator = 0; contactIterator < NumberOfContacts; contactIterator++) {
        returnVec(NumberOfContacts + contactIterator) = proxCN(lambda(contactIterator) - r * gap(contactIterator));
        if (INFO) {
          cout << "returnVec(" << NumberOfContacts + contactIterator << ")=" << proxCN(lambda(contactIterator) - r * gap(contactIterator)) << "- " << lambda(contactIterator) << endl;
          cout << "proxCN(lambda(i) - r * gap(i)) = proxCN( " << lambda(contactIterator) << "-" << r << "*" << gap(contactIterator) << endl;
        }
      }
    }

    if (INFO)
      cout << "returnVec is: " << returnVec << endl;

    return returnVec;

  }

  /* END - MaxwellFunction   ***************************************************************************************************************************/

  /* MaxwellJacobian   ***************************************************************************************************************************/

  MaxwellJacobian::MaxwellJacobian() {

  }

  MaxwellJacobian::~MaxwellJacobian() {

  }

  SqrMat MaxwellJacobian::operator ()(const Vec &distance, const void *) {
    SqrMat returnMat(3, INIT, 0.);

    return returnMat;
  }

/* END - MaxwellJacobian   ***************************************************************************************************************************/
}


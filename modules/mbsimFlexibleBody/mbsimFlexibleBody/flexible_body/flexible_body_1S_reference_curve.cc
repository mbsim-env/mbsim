/* Copyright (C) 2004-2015 MBSim Development Team
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
 * Created on: Jul 30, 2013
 * Contact: kilian.grundl@gmail.com
 */

#include "config.h"

#include "flexible_body_1S_reference_curve.h"
#include <mbsimFlexibleBody/flexible_body/finite_elements/finite_element_1S_reference_curve.h>

#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
#include <mbsim/utils/eps.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  ReferenceCurve::ReferenceCurve() : length(0) {
  }

  ReferenceCurve::~ReferenceCurve() {
  }

  FlexibleBody1SReferenceCurve::FlexibleBody1SReferenceCurve(const string & name, ReferenceCurve * refCurve) :
      FlexibleBodyContinuum<double>(name), refCurve(refCurve), elements(0.), nodeDoFs(0.), elementOrder(0.), useSpatialReferenceKinematics(false), length(0.), rho(0.), A(0.), E(0.), nu(0.), In(0.), Ib(0.), It(0.), dPulley(0.), b(Vec3("[0;0;1]")), dTheta(0.), updateReferenceEvery(0), referenceNotUpdated(updateReferenceEvery), tLastRefUpdate(0.) {

    if (0) {
      // Write data to files for testing
      ofstream myfile;
      myfile.open("h.txt", ios::trunc);
      myfile.close();

      myfile.open("M.txt", ios::trunc);
      myfile.close();

      myfile.open("dWndqk.txt", ios::trunc);
      myfile.close();

      myfile.open("test.txt", ios::trunc);
      myfile.close();
    }

  }

  FlexibleBody1SReferenceCurve::~FlexibleBody1SReferenceCurve() {
    delete refCurve;
  }

  void FlexibleBody1SReferenceCurve::init(Element::InitStage stage) {

    if (stage == preInit) {
      refCurve->computeReference();
    }

    else if (stage == resize) {
      qSize = 2; //s and theta
      double l0 = length / elements;
      int curGlobalDof = 2;
      discretization.clear();
      for (int i = 0; i < elements; i++) {
        Vec2 alpha;
        alpha(0) = i * l0;
        alpha(1) = (i + 1) * l0;
        if (i == elements - 1) {
          alpha(1) = length;
        }
        FlexibleBody1SReferenceCurveFE * ele = new FlexibleBody1SReferenceCurveFE(this, i, alpha, elementOrder, nodeDoFs);
        discretization.push_back(ele);

        qSize += (ele->getAddDoFSizeMax() - nodeDoFs * 2);
        
        /* reference the local to the global DoFs */
        VecInt dofDirElement(2 + ele->getAddDoFSizeMax(), INIT, -1);
        dofDirElement(0) = 0;
        dofDirElement(1) = 1;

        // HERMITE-REFERENCING
        if (1) {
          if (elementOrder == 3) {

            if (0) {
              //DEBUG for locked DoFs
              cout << "Locked DoFs for this ring, with at max " << 2 + elements * ele->getAddDoFSizeMax() / 2 << " Dofs are: ";
              for (set<int>::iterator i = lockedDofsFull.begin(); i != lockedDofsFull.end(); i++) {
                cout << *i << ",";
              }
              cout << endl;
            }

            // This holds for elements where also the first derivatives are shared at the element interface (Hermite elements of first order)
            int startLoop = 2;
            if (i > 0) {
              for (int localDofDir = 2; localDofDir < 2 + nodeDoFs * 2; localDofDir++) {
                dofDirElement(localDofDir) = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i - 1])->getDofDirs()(ele->getAddDoFSizeMax() - nodeDoFs * 2 + localDofDir);
              }
              startLoop = 2 + nodeDoFs * 2;
            }

            for (int localDofDir = startLoop; localDofDir < 2 + ele->getAddDoFSizeMax(); localDofDir++) {
              int globalDofDirFull = i * ele->getAddDoFSizeMax() / 2 + localDofDir;
              if (i < elements - 1 or localDofDir < 2 + ele->getAddDoFSizeMax() - nodeDoFs * 2) {
                if (std::find(lockedDofsFull.begin(), lockedDofsFull.end(), globalDofDirFull) == lockedDofsFull.end()) {
                  // if global the global DOF direction is not locked
                  dofDirElement(localDofDir) = curGlobalDof;
                  curGlobalDof++;
                }
              }
              else {
                // Ring closure
                if (i != 0) {
                  dofDirElement(localDofDir) = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[0])->getDofDirs()(localDofDir - (ele->getAddDoFSizeMax() - nodeDoFs * 2));
                }
                else {
                  // in case there is only one element (e.g. for testing the reference movement)
                  dofDirElement(localDofDir) = dofDirElement(localDofDir - (ele->getAddDoFSizeMax() - nodeDoFs * 2));
                }
              }
            }
            ele->setDofDir(dofDirElement);
          }
          else if (elementOrder == 5) {
            throw MBSimError("This is not implemented for hermite-order 5... --> Fix it now!");
            if (0) {
              //DEBUG for locked DoFs
              cout << "Locked DoFs for this ring, with at max " << 2 + elements * ele->getAddDoFSizeMax() / 2 << " Dofs are: ";
              for (set<int>::iterator i = lockedDofsFull.begin(); i != lockedDofsFull.end(); i++) {
                cout << *i << ",";
              }
              cout << endl;
            }

            // This holds for elements where also the first derivatives are shared at the element interface (Hermite elements of first order)
            int startLoop = 2;
            if (i > 0) {
              for (int localDofDir = 2; localDofDir < 2 + nodeDoFs * 2; localDofDir++) {
                dofDirElement(localDofDir) = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i - 1])->getDofDirs()(ele->getAddDoFSizeMax() - nodeDoFs * 2 + localDofDir);
              }
              startLoop = 2 + nodeDoFs * 2;
            }

            for (int localDofDir = startLoop; localDofDir < 2 + ele->getAddDoFSizeMax(); localDofDir++) {
              int globalDofDirFull = i * ele->getAddDoFSizeMax() / 2 + localDofDir;
              if (i < elements - 1 or localDofDir < 2 + ele->getAddDoFSizeMax() - nodeDoFs * 2) {
                if (std::find(lockedDofsFull.begin(), lockedDofsFull.end(), globalDofDirFull) == lockedDofsFull.end()) {
                  // if global the global DOF direction is not locked
                  dofDirElement(localDofDir) = curGlobalDof;
                  curGlobalDof++;
                }
              }
              else {
                // Ring closure
                if (i != 0) {
                  dofDirElement(localDofDir) = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[0])->getDofDirs()(localDofDir - (ele->getAddDoFSizeMax() - nodeDoFs * 2));
                }
                else {
                  // in case there is only one element (e.g. for testing the reference movement)
                  dofDirElement(localDofDir) = dofDirElement(localDofDir - (ele->getAddDoFSizeMax() - nodeDoFs * 2));
                }
              }
            }
            ele->setDofDir(dofDirElement);
          }
          else {
            throw MBSimError("This is not implemented for other hermite orders... --> Fix it now!");
          }
        }

        // testing the indices
        if (0)
          cout << "These are the DoF direction of Element \"" << i + 1 << "\" " << dofDirElement << endl;

      }

      // remove these DOFs from the qSize again
      qSize -= lockedDofsFull.size();

      // set the size of the generalized velocities
      uSize[0] = qSize;
      uSize[1] = qSize;

      //set the sizes for the PT1-controlled DOFs
      //qF.resize(qSize, INIT, 0.);
      //uF.resize(uSize[0], INIT, 0.);
      lambdaqF.resize(qSize, INIT, -1);
      lambdauF.resize(uSize[0], INIT, -1);

      // initialize the switches if these are not given (size is zero)
      if (lambdaqFSwitches.size() == 0)
        lambdaqFSwitches.resize(getEleDofs() / 2 + 2, INIT, -1);
      if (lambdauFSwitches.size() == 0)
        lambdauFSwitches.resize(getEleDofs() / 2 + 2, INIT, -1);

      if (lambdaqFSwitches.size() != getEleDofs() / 2 + 2)
        cout << "WARNING: The given lambda-switches for q are not set correctly for the given order of \"" << elementOrder << "\" and the given node Dofs of \"" << nodeDoFs << "\". It is therefore disabled!";
      if (lambdauFSwitches.size() != getEleDofs() / 2 + 2)
        cout << "WARNING: The given lambda-switches for u are not set correctly for the given order of \"" << elementOrder << "\" and the given node Dofs of \"" << nodeDoFs << "\". It is therefore disabled!";

// set the sizes for the different parts in the h-vector
//      hT1.resize(uSize[0], INIT, 0.);
//      hT2.resize(uSize[0], INIT, 0.);
//      hT3.resize(uSize[0], INIT, 0.);
//      hV123.resize(uSize[0], INIT, 0.);
//      hV4.resize(uSize[0], INIT, 0.);

      if (0) {
        // Add directions to h-file
        ofstream myfile;
        myfile.open("h.txt", ios::app);
        myfile.precision(2);
        myfile << "[             s";
        myfile << "          Theta";
        for (int i = 2; i < qSize; i++) {
          int ind = i - 2;
          if (std::find(lockedDofsFull.begin(), lockedDofsFull.end(), i) == lockedDofsFull.end()) {
            if (ind % 4 == 0) {
              myfile << "             t";
            }
            else if (ind % 4 == 1) {
              myfile << "             n";
            }
            if (ind % 4 == 2) {
              myfile << "            t'";
            }
            if (ind % 4 == 3) {
              myfile << "            n'";
            }
          }
        }
        myfile << endl;
        myfile.close();
      }

    }
    else if (stage == plotting) {
      updatePlotFeatures();
      if (getPlotFeature(plotRecursive) == enabled) {
        if (getPlotFeature(notMinimalState) == enabled) {
          for (int i = 0; i < qSize; ++i)
            plotColumns.push_back("qF(" + numtostr(i) + ")");
          for (int i = 0; i < uSize[0]; ++i)
            plotColumns.push_back("uF(" + numtostr(i) + ")");
        }
      }
    }
    else if (stage == unknownStage) {

      for (size_t i = 0; i < discretization.size(); i++) {
        FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
        // For this body qElement and uElement hold the DoFs of the Finite element with the reference DOFs.
        // Remark: the values of possible locked dofs are not saved  in this vector
        //TODO: if Dofs would be locked at a different value as zero this could/must be changed --> probably affects the whole structure though
        qElement.push_back(Vec(ele->getFreeDofs().size(), INIT, 0.));
        uElement.push_back(Vec(ele->getFreeDofs().size(), INIT, 0.));
        qElementAll.push_back(Vec(ele->getDofDirs().size(), INIT, 0.));
        uElementAll.push_back(Vec(ele->getDofDirs().size(), INIT, 0.));
      }

// initialize the lamdaq and the lamdau vectors**/
// Firstly the reference dofs
      lambdaqF(0) = lambdaqFSwitches(0);
      lambdaqF(1) = lambdaqFSwitches(1);
      lambdauF(0) = lambdauFSwitches(0);
      lambdauF(1) = lambdauFSwitches(1);

// the current index for setting the lambda values
      int curInd = 2;
// loop over all possible DOFs
      for (size_t indFull = 2; indFull < qSize + lockedDofsFull.size(); indFull++) {
        // TODO initialize the lambdaq/u switches! --> problematic because of the locked dofs --> see output for h-vector
        // search if index is not blocked
        if (std::find(lockedDofsFull.begin(), lockedDofsFull.end(), indFull) == lockedDofsFull.end()) {
          // find the direction that is affected
          int dir = ((indFull - 2) % (nodeDoFs * (elementOrder - 1))) + 2;
          lambdaqF(curInd) = lambdaqFSwitches(dir);
          lambdauF(curInd) = lambdauFSwitches(dir);
          curInd++;  // increase index if it was valid
        }
      }

      if (0) {
        // debug-output for the lambda-factors
        cout << "lambdaqFSw" << lambdaqFSwitches.T() << endl;
        cout << "lambdaqF" << lambdaqF.T() << endl;
        cout << "lambdauFSw" << lambdauFSwitches.T() << endl;
        cout << "lambdauF" << lambdauF.T() << endl;
      }

        //TODO_Grundl: get the following into the sub-reference curve!
//      if (0) {
//
//        // Test reference
//        int pnts = 50;
//        Vec x(pnts, NONINIT);
//        Vec y(pnts, NONINIT);
//
//        for (double theta = refCurve->getThetaLower(); theta < refCurve->getThetaUpper(); theta += 1e-1) {
//          for (int i = 0; i < pnts; i++) {
//            double xi = 0. + i * length / (pnts - 1);
//            Vec3 vec(refCurve->computeVecAt(xi, theta, 0, 0));
//            x(i) = vec(0);
//            y(i) = vec(1);
//          }
//          cout << "theta = " << theta;
//          cout << x << endl << y << endl;
//
//        }
//      }
    }

// initialize elements
    for (size_t i = 0; i < discretization.size(); i++) {
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
      ele->init(stage);
    }

    FlexibleBodyContinuum<double>::init(stage);
  }

  void FlexibleBody1SReferenceCurve::initInfo(Vec q0, Vec u0) {
    init(preInit);
    init(resize);
    init(unknownStage);

    // initialize the DOFs
    if (q0.size() == 0)
      setq0(Vec(qSize, INIT, 0));
    else
      setq0(q0);
    if (u0.size() == 0)
      setu0(Vec(uSize[0], INIT, 0));
    else
      setu0(u0);

    // update the elements
    BuildElements(0);
  }

  void FlexibleBody1SReferenceCurve::updatedq(double t, double dt) {
    qd = uF * dt; //T is the identity here
    /* update the controlled values (if a frequency, i.e. lambda value, is given */
    for (int i = 0; i < qSize; i++) {
      if (lambdaqF(i) < 0) {
        qF(i) = q(i) + qd(i);
      }
// for the integration two possible (internal) integrators are used: the explicit or the implicit euler rule
      else {
        if (0) {
          // explicit writing
          qF(i) += dt * lambdaqF(i) * (q(i) - qF(i));
        }
        else {
          // implicit writing
          double a = 1 / (lambdaqF(i) * dt);
          double Tstar = 1 / (1 + a);
          qF(i) += Tstar * ((q(i) + qd(i)) - qF(i));
        }
      }
    }
  }

  void FlexibleBody1SReferenceCurve::updatedu(double t, double dt) {
    ud[0] = slvLLFac(LLM[0], h[0] * dt + r[0]);
    /* update the controlled values (if a frequency, i.e. lambda value, is given */
    for (int i = 0; i < uSize[0]; i++) {
      if (lambdauF(i) < 0) {
        uF(i) = u(i) + ud[0](i);
      }
      else {
        if (0) {
          // explicit writing
          uF(i) += dt * lambdauF(i) * (u(i) - uF(i));
        }
        else {
          // implicit writing (after wikipedia http://de.wikipedia.org/wiki/PT1-Glied)
          double a = 1 / (lambdauF(i) * dt);
          double Tstar = 1 / (1 + a);
          uF(i) += Tstar * ((u(i) + ud[0](i)) - uF(i));
        }
      }
    }
  }
  void FlexibleBody1SReferenceCurve::updateud(double t, int i) {
    if (t <= 0.)
      Atom::msgStatic(Atom::Warn) << "Not implemented correctly: " << __func__ << endl << " does not consider the internal \"lambda\"-transformation..." << endl;
    FlexibleBodyContinuum<double>::updateud(t, i);
  }

  void FlexibleBody1SReferenceCurve::updateqd(double t) {
    if (t <= 0.)
      Atom::msgStatic(Atom::Warn) << "Not implemented correctly: " << __func__ << endl << " does not consider the internal \"lambda\"-transformation..." << endl;
    FlexibleBodyContinuum<double>::updateqd(t);
  }

  void FlexibleBody1SReferenceCurve::initz() {
    FlexibleBodyContinuum<double>::initz();
    qF = q.copy();
    uF = u.copy();
  }

  void FlexibleBody1SReferenceCurve::plot(double t, double dt) {
    if (getPlotFeature(plotRecursive) == enabled) {
      if (getPlotFeature(notMinimalState) == enabled) {
        for (int i = 0; i < qSize; ++i)
          plotVector.push_back(qF(i));
        for (int i = 0; i < uSize[0]; ++i)
          plotVector.push_back(uF(i));
      }
    }
    FlexibleBodyContinuum<double>::plot(t, dt);
  }

  void FlexibleBody1SReferenceCurve::BuildElements(double t) {
    for (int eleNo = 0; eleNo < elements; eleNo++) {
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
      vector<int> freeDofs = ele->getFreeDofs();
      VecInt dofDirs = ele->getDofDirs();
      for (size_t dofdir = 0; dofdir < freeDofs.size(); dofdir++) {
        int globInd = dofDirs(freeDofs[dofdir]);
        if (globInd >= 0) { //TODO: avoid if here --> put it into a map or something!
          //TODO: really nasty with this two vectors... --> actually qElementAll should be used for general description, however than not the nice operators of fmatvec can be used in updateh-routes (e.g.) as then the sizes don't fit anymore and the loops would have to be written specifically for this problem. I don't invest the time atm.
          if (not usePT1) {
            qElement[eleNo](dofdir) = q(globInd);
            uElement[eleNo](dofdir) = u(globInd);
            qElementAll[eleNo](freeDofs[dofdir]) = q(globInd);
            uElementAll[eleNo](freeDofs[dofdir]) = u(globInd);
          }
          else {
            qElement[eleNo](dofdir) = qF(globInd);
            uElement[eleNo](dofdir) = uF(globInd);
            qElementAll[eleNo](freeDofs[dofdir]) = qF(globInd);
            uElementAll[eleNo](freeDofs[dofdir]) = uF(globInd);
          }
        }
      }

      if (0) {
        cout << "EleNo = " << eleNo << endl;
        cout << qElement[eleNo].T() << endl;
        cout << qElementAll[eleNo].T() << endl;
      }

// update the borders for the integrals
      ele->updateIntegrationBorders(0.);
    }
  }

  void FlexibleBody1SReferenceCurve::updateh(double t, int k) {
// Remark: as the
    if (1) {
      for (int i = 0; i < (int) discretization.size(); i++)
        discretization[i]->computeh(qElement[i], uElement[i]); // compute attributes of finite element
      for (int i = 0; i < (int) discretization.size(); i++)
        GlobalVectorContribution(i, discretization[i]->geth(), h[k]); // assemble

      VecV dampedMasses(M[k].cols(), NONINIT);
      if (not usePT1) {
        dampedMasses = M[k] * u;
      }
      else {
        dampedMasses = M[k] * uF;
      }

      h[k](1) -= dTheta * dampedMasses(1);

      for (int i = 2; i < h[k].size(); i++)
        h[k](i) -= d_massproportional * dampedMasses(i);
    }

    if (referenceNotUpdated < updateReferenceEvery) {
// set all forces acting in the reference direction to be zero
      h[k](0) = 0.;
      h[k](1) = 0.;
    }
    else {
      referenceNotUpdated = 0;
    }

    if (t > tLastRefUpdate) {
      referenceNotUpdated++;
      tLastRefUpdate = t;
    }

    // Write debug output to files
    ofstream myfile;
    if (0) {
      myfile.open("dWndqk.txt", ios::app);
      myfile.precision(2);
      for (int eleNo = 0; eleNo < 1; eleNo++) {
        FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
        myfile << "dWndqk3 = [ ";
        for (double xi = ele->getAlpha()(0); xi <= ele->getAlpha()(1); xi += (ele->getAlpha()(1) - ele->getAlpha()(0)) / 100.) {
          myfile << ele->dWndqk(xi, 3) << ", ";
        }
        myfile << "];" << endl;
        myfile << "dWndqk5 = [ ";
        for (double xi = ele->getAlpha()(0); xi <= ele->getAlpha()(1); xi += (ele->getAlpha()(1) - ele->getAlpha()(0)) / 100.) {
          myfile << ele->dWndqk(xi, 5) << ", ";
        }
        myfile << "];" << endl;
        myfile << "dWndqk7 = [ ";
        for (double xi = ele->getAlpha()(0); xi <= ele->getAlpha()(1); xi += (ele->getAlpha()(1) - ele->getAlpha()(0)) / 100.) {
          myfile << ele->dWndqk(xi, 7) << ", ";
        }
        myfile << "];" << endl;
        myfile << "dWndqk9 = [ ";
        for (double xi = ele->getAlpha()(0); xi <= ele->getAlpha()(1); xi += (ele->getAlpha()(1) - ele->getAlpha()(0)) / 100.) {
          myfile << ele->dWndqk(xi, 9) << ", ";
        }
        myfile << "];" << endl;
      }
      myfile << endl;
      myfile.close();

      exit(1);
    }

    if (0) {
      myfile.open("h.txt", ios::app);
      myfile.precision(2);
      myfile << h[k].T() << endl;
      myfile.close();
    }
  }

  void FlexibleBody1SReferenceCurve::updateM(double t, int k) {
    if (1) {
      FlexibleBodyContinuum<double>::updateM(t, k);

//TODO: just for testing circular reference
      if (0) {
        for (int i = 0; i < M[k].cols(); i++) {
          M[k](i, 0) = 0;
          M[k](i, 1) = 0;
        }
        M[k](0, 0) = 1;
        M[k](1, 1) = 1;
      }

//TODO: just for testing only reference movement
      if (0) {
        for (int i = 2; i < M[k].cols(); i++) {
          M[k](i, 0) = 0;
          M[k](i, 1) = 0;
          h[k](i) = 0;
          M[k](i, i) = 1;
          for (int j = i + 1; j < M[k].cols(); j++) {
            M[k](i, j) = 0;
          }
        }
      }

    }

    if (0) {
      ofstream myfile;
      myfile.open("h.txt", ios::app);
      myfile.precision(2);
      myfile << "after links...:" << h[k].T() << endl;
      myfile.close();
    }

    if (0) {
      ofstream myfile;
      myfile.open("M.txt", ios::app);
      myfile.precision(3);
      myfile << M[k] << endl;
      myfile.close();
    }
  }

  void FlexibleBody1SReferenceCurve::GlobalVectorContribution(int eleNo, const fmatvec::Vec& eleVec, fmatvec::Vec& globalVec) {
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
    std::vector<int> freeDofs = ele->getFreeDofs();
    VecInt dofDirs = ele->getDofDirs();
    for (size_t i = 0; i < freeDofs.size(); i++) {
      int globalDof = dofDirs(freeDofs[i]);
      globalVec(globalDof) += eleVec(i);
    }
  }

  void FlexibleBody1SReferenceCurve::GlobalMatrixContribution(int eleNo, const fmatvec::Mat3xV& eleMat, fmatvec::Mat3xV& globMat) {
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
    std::vector<int> freeDofs = ele->getFreeDofs();
    VecInt dofDirs = ele->getDofDirs();
    for (size_t i = 0; i < freeDofs.size(); i++) {
      int globalDof = dofDirs(freeDofs[i]);
      globMat.add(globalDof, eleMat.col(i));
    }
  }

  void FlexibleBody1SReferenceCurve::GlobalMatrixContribution(int eleNo, const fmatvec::SymMat& eleMat, fmatvec::SymMat& globMat) {
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
    std::vector<int> freeDofs = ele->getFreeDofs();
    VecInt dofDirs = ele->getDofDirs();
    for (size_t row = 0; row < freeDofs.size(); row++) {
      int globalDofRow = dofDirs(freeDofs[row]);
      for (size_t col = row; col < freeDofs.size(); col++) {
        int globalDofCol = dofDirs(freeDofs[col]);
        globMat(globalDofRow, globalDofCol) += eleMat(row, col);
      }
    }
  }

//  void FlexibleBody1SReferenceCurve::updateKinematicsForFrame(MBSim::ContourPointData& cp, MBSim::Frame::Feature fFeature, MBSim::Frame* frame) {
//    //TODO: use the special neutral contour here for computation (more current concept) and not the old concept
//    if (fFeature == Frame::position) {
//      double xBar = cp.getLagrangeParameterPosition()(0);
//      double xi = xBar - q(0);
//      cp.getFrameOfReference().setPosition(computer(xi, 0, 0));
//    }
//    else {
//      throw MBSim::MBSimError("NOT IMPLEMENTED: " + std::string(__func__));
//    }
//  }
//
//  void FlexibleBody1SReferenceCurve::updateStateDependentVariables(double t) {
//    //TODO_Grundl: Move the following to specific case!
////    if (0) {
//////Remark: artificially updating Theta to test output from lower to upper bound
////      q.init(0.);
////      q(1) = refCurve->getThetaLower() + (refCurve->getThetaUpper() - refCurve->getThetaLower()) * t / 1e-4;
////
//////      double amp = 1e-2;
//////      double amplitude = -amp + (amp + amp) * t / 1e-4;
//////       first "major" node is locked in t/n direction ...
//////      for (int i = 3; i < qSize; i += nodeDoFs * 2) {
//////        q(i) = amplitude;          // * sin(double(i) / (elements * elementOrder) * 2. * M_PI);
//////        if (i < 4) {
//////          i -= 1;
//////        }
//////      }
//////      for (int i = 3; i < qSize; i += nodeDoFs * 2) {
//////        q(i) = amplitude * sin(double(i) / (elements * elementOrder) * 2. * M_PI);
//////      }
//////      for (int i = 4; i < qSize; i += nodeDoFs * 2) {
//////        q(i) = amplitude * sin(double(i) / (elements * elementOrder) * 2. * M_PI);
//////      }
////    }
//
//    FlexibleBody::updateStateDependentVariables(t);
//
//    if (0) {
//      updateM(0.0, 0);
//      updateh(0.0, 0);
//      ofstream myfile;
//      myfile.open("test.txt", ios::app);
//      myfile.precision(10);
//      myfile << q(1) << "," << h[0](1) << "," << M[0](1, 0) << "," << M[0](1, 1) << ";";
//      myfile.close();
//    }
//
//    if (0) {
//// test basically the curvature energy-functions
//      int n = 1000;
//      Vec kappa(n, NONINIT);
//      Vec dkappadq(n, NONINIT);
//      for (int i = 0; i < n; i++) {
//        double xi = i * length / (n + 1);
//        int eleNo = findElement(xi);
//        FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
//        kappa(i) = ele->computeKappan(xi);
//        dkappadq(i) = ele->computedKappandqk(xi, 9);
//      }
//      cout << kappa.T() << ";" << endl;
//      cout << dkappadq.T() << ";" << endl;
//      exit(0);
//    }
//  }
//
//  void FlexibleBody1SReferenceCurve::updateKinematicsAtNode(MBSimFlexibleBody::NodeFrame *frame, MBSim::Frame::Feature ff) {
//    int node = frame->getNodeNumber();
//    double lagrangePos = node * length / elements;
//    bool useRef = false;
//
//// Translational node
//    if (ff == Frame::position || ff == Frame::position_cosy || ff == Frame::all) {
//      if (useRef)
//        frame->setPosition(computerRef(lagrangePos, 0, 0));
//      else
//        frame->setPosition(computer(lagrangePos, 0, 0));
//    }
//    if (ff == Frame::velocity || ff == Frame::velocity_cosy || ff == Frame::velocities || ff == Frame::velocities_cosy || ff == Frame::all)
//      frame->setVelocity(computev(lagrangePos));
//
//    if (ff == Frame::normal || ff == Frame::firstTangent || ff == Frame::secondTangent || ff == Frame::cosy || ff == Frame::position_cosy || ff == Frame::velocity_cosy || ff == Frame::velocities_cosy || ff == Frame::all) {
//      if (useRef)
//        frame->setOrientation(computeARef(lagrangePos, 0, 0));
//      else {
//        Vec3 rdXi = computer(lagrangePos, 1, 0);
//        Vec3 t = rdXi / nrm2(rdXi);
//        // the tangent does not change -> the normal vector is not uniquely defined! --> chose normal that point outwards...
//        Vec3 n = crossProduct(t, -b);
//        frame->getOrientation().set(0, n);
//        frame->getOrientation().set(1, t);
//        frame->getOrientation().set(2, crossProduct(n, t));
//      }
//    }
//
////TODO: more infomation needed after all?
//    if (ff == Frame::velocities || ff == Frame::velocity_cosy || ff == Frame::velocities_cosy || ff == Frame::all) {
//      frame->getVelocity() = computev(lagrangePos);
//      frame->getAngularVelocity() = Vec3(); // TODO
//    }
//  }

  Contour1sNeutralFlexibleBody1SReferenceCurve* FlexibleBody1SReferenceCurve::createNeutralPhase(const std::string & contourName) {
// add neutral contour to the rod
    Contour1sNeutralFlexibleBody1SReferenceCurve* ncc = new Contour1sNeutralFlexibleBody1SReferenceCurve(contourName);

    std::vector<double> nodes(elements + 1);
    for (int i = 0; i < elements + 1; i++) {
      nodes[i] = i * length / (elements);
    }
    ncc->setFrameOfReference(getFrameOfReference());
    ncc->setAlphaStart(0);
    ncc->setAlphaEnd(length);
    ncc->setNodes(nodes);
    addContour(ncc);

    return ncc;
  }

  double FlexibleBody1SReferenceCurve::computePhysicalStrain(double xi) {
    double eps;
    double factor = 1000.;
    double xi1 = xi - length / factor;
    double xi2 = xi + length / factor;
    if (xi < length / factor) {
      xi1 = length / factor;
      xi2 = 99. * length / factor;
    }
    Vec3 t1 = computer(xi1, 1, 0); // tangential direction of ring
    double eps1 = sqrt(scalarProduct(t1, t1)) - 1.;
    Vec3 t2 = computer(xi2, 1, 0); // tangential direction of ring
    double eps2 = sqrt(scalarProduct(t2, t2)) - 1.;
    eps = (eps1 + eps2) / 2.;
    return eps;
  }

  Vec FlexibleBody1SReferenceCurve::computeNeutralState(const Vec & q) {
    MultiDimensionalNewtonMethod newton;
    gethFunc fct(this);
    NumericalNewtonJacobianFunction numJac;
    GlobalResidualCriteriaFunction criteria;
    StandardDampingFunction damping;
    newton.setMaximumNumberOfIterations(1000);
    criteria.setTolerance(10);
    damping.setMaximalDampingSteps(100);

    newton.setFunction(&fct);
    newton.setCriteriaFunction(&criteria);
    newton.setJacobianFunction(&numJac);
//    newton.setDampingFunction(&damping);
    Vec res(newton.solve(q));
    cout << newton.getInfo() << endl;
    cout << newton.getNumberOfIterations() << endl;
    return res;

  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computer(double xi, int derXi, int derTheta) {
    int eleNo = findElement(xi);
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
    return ele->computer(xi, derXi, derTheta);
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computedrdqk(double xi, int derXi, int qInd) {
    Vec3 drdqk;
    if (qInd == 0) {
// stays zero
    }
    else if (qInd == 1) {
      drdqk = computer(xi, derXi, 1);
    }
    else {
      int eleNo = findElement(xi);
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
      int qIndLoc = ele->findLocalDof(qInd);
      drdqk = ele->computeB(qIndLoc, xi, derXi, 0);
    }

    return drdqk;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computev(double xi) {
    int eleNo = findElement(xi);
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[eleNo]);
    return ele->computev(xi, 0);
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computerRef(double xi, int derXi, int derTheta) {
    Vec3 r;

    if (not usePT1)
      r = refCurve->computeVecAt(xi, q(1), derXi, derTheta);
    else
      r = refCurve->computeVecAt(xi, qF(1), derXi, derTheta);
    ;
// TODO: This formula forces the first derivative of the position to have the length one. Of course this is not okay --> there should be an optimization of the reference curve computation to achieve this goal!
//    if(derXi >= 1) {
//    	r /= nrm2(refCurve.computeVecAt(xi, q(1), 1, derTheta));
//    }
    return r;
  }

  fmatvec::SqrMat3 FlexibleBody1SReferenceCurve::computeARef(double xi, int derXi, int derTheta) {
    SqrMat3 A;
    Vec3 t = computerRef(xi, 1, derTheta);
    double len = nrm2(t);
    if (derXi == 0) {
      if (len > macheps()) {
        // if len is zero than that means that the tangential does no change at the xi-position with theta (i.e. at xi = 0 and xi = length/2)
        t = t / len;
      }
    }
    else {
      Vec3 rRefdxidxi = computerRef(xi, derXi, derTheta);
      if (len > macheps()) {
        // if len is zero than that means that the tangential does no change at the xi-position with theta (i.e. at xi = 0 and xi = length/2)
        t = rRefdxidxi / len;
      }
//      t -= t * scalarProduct(t, rRefdxidxi) / len / len / len; // scalar product is zero for all derivatives
    }

    A.set(0, t);
    A.set(1, crossProduct(b, t));
    if (derXi + derTheta == 0) {
      A.set(2, b);
    }

    return A;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computenRef(double xi, int derXi, int derTheta) {
    Vec3 t = computerRef(xi, 1, derTheta);
    double len = nrm2(t);
    if (derXi == 0) {
      t = t / len;
    }
    else {
      Vec3 rRefdxidxi = computerRef(xi, derXi, derTheta);
      t = rRefdxidxi / len;
//      t -= t * scalarProduct(t, rRefdxidxi) / len / len / len; // scalar product is zero for all derivatives
    }

    return crossProduct(b, t);
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computeSElement(int globalDOF, double xi, int derXi) {
    Vec3 ret;
    vector<pair<int, int> > eleLocDof = getElementAndLocalDoFNo(globalDOF);
    for (size_t i = 0; i < eleLocDof.size(); i++) {
      int elementNo = eleLocDof[i].first;
      int localDof = eleLocDof[i].second;
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[elementNo]);
      ret += ele->computeS(localDof, xi, derXi);
    }
    return ret;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computeSqfElement(double xi, int derXi) {
    int element = findElement(xi);
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[element]);
    Vec3 ret = ele->computeSTimeslocVec(xi, derXi, qElementAll[element]);
    return ret;
  }

  fmatvec::Vec3 FlexibleBody1SReferenceCurve::computeSufElement(double xi, int derXi) {
    int element = findElement(xi);
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[element]);

    return ele->computeSTimeslocVec(xi, derXi, uElementAll[element]);
  }

  fmatvec::Mat3xV FlexibleBody1SReferenceCurve::computeP(double xi, int derXi) {
    Mat3xV P(qSize, INIT, 0.);

    int i = findElement(xi);
    FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
    GlobalMatrixContribution(i, ele->computeP(xi, derXi), P);

    return P;
  }

  fmatvec::Mat3xV FlexibleBody1SReferenceCurve::computedPdqk(double xi, int qInd) {
    Mat3xV dPdqk(qSize, INIT, 0.);

    for (int i = 0; i < elements; i++) {
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
      int qIndLoc = ele->findLocalDof(qInd);
      GlobalMatrixContribution(i, ele->computedPdqk(xi, qIndLoc), dPdqk);
    }

    return dPdqk;
  }

  int FlexibleBody1SReferenceCurve::findElement(double & xi) {
    while (xi > length)
      xi -= length;
    while (xi < 0.)
      xi += length;

    FlexibleBody1SReferenceCurveFE * ele;
    for (int i = 0; i < elements; i++) {
      ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
      if (xi >= ele->getAlpha()(0) and xi <= ele->getAlpha()(1))
        return i;
    }
    return -1;
  }

  std::vector<std::pair<int, int> > FlexibleBody1SReferenceCurve::getElementAndLocalDoFNo(int globalDoFNo) {
    throw MBSimError("Still figuring out if this is used ..." + string(__func__));
    std::vector<std::pair<int, int> > ret;
    for (int i = 0; i < elements; i++) {
      FlexibleBody1SReferenceCurveFE * ele = static_cast<FlexibleBody1SReferenceCurveFE*>(discretization[i]);
      VecInt dofDirEle = ele->getDofDirs();
      for (int j = 0; j < dofDirEle.size(); j++) {
        if (dofDirEle(j) == globalDoFNo) {
          ret.push_back(pair<int, int>(i, j));
        }
      }
    }

    return ret;
  }

  int FlexibleBody1SReferenceCurve::getEleDofs() const {
    return nodeDoFs * (elementOrder - 1) * 2;
  }

}


#include "system.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematic_functions.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/polygonpoint.h>
#endif

#include <mbsim/utils/eps.h>
#include <mbsim/utils/stopwatch.h>

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

void Perlchain::updateG(double t, int j) {

//    cout << "DynamicSystemSolver::updateG(): W[" << j<< "]: "<< W[j] << endl;
//    cout << "DynamicSystemSolver::updateG(): LLM[" << j<< "]: "<< LLM[j] << endl;
//    cout << "DynamicSystemSolver::updateG(): V[" << j<< "]: "<< V[j] << endl;
//  if (j == 0) {
//    static int ITER = 0;
//    std::ostringstream strs;
//    strs << ITER++;
//    std::string str = strs.str();
//    ofstream WOfile(str.c_str());
//    WOfile << LLM[j];
//    WOfile.close();
//  }

//     G << SqrMat(W[j].T() * slvLLFac(LLM[j], V[j])); //TODO@zhan: this is the line!
//     ofstream WOfile("G_original.txt");
//     WOfile << G;
//     WOfile.close();

//  int I1; // the ending row index of the dense matrix area
//  int numOfDenseObjects;
  StopWatch sw;

  bool comparison = false;

  if (comparison) {
    sw.start();
  }
  cs *cs_L_LLM, *cs_Wj;
//  numOfDenseObjects = 3;

//REMARK: What you do here, you could do that with every object. The objects in the CVT will then return 3x3 matrix, where you will check for non-zero values also. Why not? Is more general and I guess does not really hurt, right? Maybe it would also be possible to give every object the possibility to return the non-zero values in a sparse format and return it here... So we could also avoid the checking every time-step.
//  int uInd = object[numOfDenseObjects - 1]->getuInd(j); // todo: to check are the dynamicssystem before the objects ?
//  int uSize = object[numOfDenseObjects - 1]->getuSize(j);

//  I1 = uInd + uSize - 1;
  int n = LLM[j].cols();
//  int nz = 5 * n + 200; // todo: add a function in symetry matrix to calculate the number of nonzero entries and initiate a static nz in the first time step.
//  int TwoD = 1; // todo: get this value from the system infromation

// the function compress only the lower triangular part of LLM
//    cs *cs_compress_LLM(const double *ele, const int lda, const int m, const int n, const int nz, const int I1, const int TwoD);

// REMARK: the compressLLM should be a function of the DS, as the knowledge about the objects is there already. It must not be implemented in a CVT-specific way, as I also don't really understand what is the profit out of it
// REMARK: nz could be guessed by the sub-block-sizes!
  cs_L_LLM = compressLLM_LToCsparse(j);

//  cs_print(cs_L_LLM, 0);

// construct the w in the triplet form here and using cs_triplet (const cs *T) to transform
// or directly transform the w
  cs_Wj = compressWToCsparse(j);

  int * xi = (int *) cs_malloc(2 * n, sizeof(int));

  Mat y(n, W[j].cols(), NONINIT);
//   double * yele = y.operator ()();
//   int ylda = y.ldim();

// solve Ly=w  todo: store the results into a vector in a safe way
  for (int k = 0; k < W[j].cols(); k++) {
    double * x = (double *) calloc(n, sizeof(double));
    //top = cs_splsolve (cs_L_LLM, cs_Wj, k, xi, yele + ylda * k , 0); // yele + ylda * k is the pointer to the kth column of y.ele todo: is it acceptable by accessing the private date of y?
    cs_splsolve(cs_L_LLM, cs_Wj, k, xi, x, 0);
    //TODO: Is "L = cs_L_LLM" here really only lower triangular? I don't see that on: http://people.sc.fsu.edu/~jburkardt/c_src/csparse/csparse.html (not that it reads always the full matrix?)

    for (int iter = 0; iter < n; iter++) {
      y(iter, k) = x[iter];
    }

    free(x);
  }

  G.resize(y.cols(), NONINIT);

  for (int i = 0; i < y.cols(); i++) {
    RowVec temp = y.col(i).T(); // temp can be kept in the fast memmory part
    for (int j = i; j < y.cols(); j++) {
      double val = temp * y.col(j);
      G(i, j) = val;
      G(j, i) = val;
    }
  }

  double t_cs = sw.stop(true);

//  ofstream LLMcmp("LLM_cmp.txt");
//  LLMcmp << LLM[j] << endl;
//  LLMcmp << cs2Mat(cs_L_LLM) << endl;
//  LLMcmp << LLM[j] - cs2Mat(cs_L_LLM);
//  LLMcmp.close();
//
//  ofstream ycsOfile("y_cs.txt");
//  ycsOfile << y;
//  ycsOfile.close();

  SqrMat Greference(0, NONINIT);
  if (comparison) {
    sw.start();
    Greference.resize() << W[j].T() * slvLLFac(LLM[j], V[j]);
  }
  double t_ref = sw.stop(true);

//  ofstream GOfile("G_cs.txt");
//  GOfile << G;
//  GOfile.close();
//
//  ofstream GOreffile("G_reference.txt");
//  GOreffile << Greference;
//  GOreffile.close();
//
//  SqrMat Gcmp(G - Greference);
//  ofstream GOcmpfile("Gcmp.txt");
//  GOcmpfile << Gcmp;
//  GOcmpfile.close();

  if (comparison) {
    cout << " *** Comparison *** " << endl;
    cout << "1-Norm of W-difference:   \t" << nrm1(W[j] - cs2Mat(cs_Wj)) << endl;
    cout << "1-Norm of LLM-difference: \t" << nrm1(LLM[j] - cs2Mat(cs_L_LLM)) << endl;
    cout << "1-Norm of G-difference:   \t" << nrm1(G - Greference) << endl;

    cout << "Timing: t_cs = " << t_cs << " s   | t_ref =" << t_ref << "s   | dt = " << (t_cs) / t_ref * 100 << " %" << endl;
  }

  // todo: declare G to be symmetric matrix
  // if G is symmetric type, only the elements in the lower triangular is stored in column wise.
  // if it is accessed in row wise order in the upper triangular part, is will be re-mapped to the lower part,
  // so ele can be still one by one in the order how they stored in memory.

//    for (int i = 0; i < y.cols(); i++){
//      RowVec temp = y.col(i).T(); // temp can be kept in the fast memmory part
//      for (int j = i; j < y.cols(); j++){
//        G(i,j) = temp * y.col(j);
//      }
//    }

  //todo: free the allocated memory by the cSparse
  cs_spfree(cs_Wj);
  cs_spfree(cs_L_LLM);
  free(xi);

//
//
//    cout << "DynamicSystemSolver::updateG(): G[" << j<< "]: "<< G;
//    // something like that:  Gs = SparseMatrix(W, LLM, V);

  if (checkGSize)
    ;  // Gs.resize();
  else if (Gs.cols() != G.size()) {
    static double facSizeGs = 1;
    if (G.size() > limitGSize && fabs(facSizeGs - 1) < epsroot())
      facSizeGs = double(countElements(G)) / double(G.size() * G.size()) * 1.5;
    Gs.resize(G.size(), int(G.size() * G.size() * facSizeGs));
  }
  Gs << G;
}

cs * Perlchain::compressWToCsparse(int j) {

  int m, n, nz; // row, column, nz;//, I1;
  cs *C;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  m = W[j].rows();
  n = W[j].cols();
  nz = 5 * m; // todo: find a method to determine the value of nz

  C = cs_spalloc(m, n, nz, 1, 1); /* allocate memory for the sparse matrix C in Triplet formate, the last input value is 1!*/
  if (!C)
    return (cs_done(C, 0, 0, 0)); /* out of memory */

  // todo: need to refine when considering the sub DynamicSystem
  // convert the upper part of W matrix (which corresponding to the DOFs of flexible belt) into cs triplet format
//  I1 = object[0]->getuInd(j) + object[0]->getuSize(j) - 1;
//  for (column = 0; column < n; column++) {
//    for (row = 0; row <= I1; row++) {  // i = 0: scan the whole rectangle area.
//      double entry = W[j](row, column);
//      if (fabs(entry) > EPSILON)
//        cs_entry(C, row, column, entry);  //REMARK: why won't you use this function in the compress_LLM-function?
//
//    }
//  }

  // convert the bottom part of W matrix into cs triplet format
//  int LinkPartner1Bottom = 0;
//  int LinkPartner2Upper = 0;
  // only consider the contour in the bottom part of W matrix
//  int LinkPartner1B = 0;
//  int LinkPartner2U = 0;

  for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
    LinkMechanics * i_temp = dynamic_cast<LinkMechanics*>(*i);
    for (int col = (**i).getlaInd(); col < (**i).getlaInd() + (**i).getlaSize(); col++) {
      if ((**i).getType() == "Joint") {  // for joint
        const size_t Noframes = (*i_temp).getFrame().size();
        for (size_t partner = 0; partner < Noframes; partner++) {
          int lowerRow = (*i_temp).getFrame()[partner]->gethInd(j);
          int upperRow = (*i_temp).getFrame()[partner]->gethInd(j) + (*i_temp).getFrame()[partner]->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON)
              cs_entry(C, row, col, entry);
          }
        }
//        LinkPartner1Bottom =;
//        LinkPartner2Upper =;
        // only consider the frame in the bottom part of W matrix
//        LinkPartner1B = max((*i_temp).getFrame()[0]->gethInd(j), (*i_temp).getFrame()[1]->gethInd(j));
//        LinkPartner2U = max((*i_temp).getFrame()[0]->gethInd(j) + (*i_temp).getFrame()[0]->getJacobianOfTranslation(j).cols(), (*i_temp).getFrame()[1]->gethInd(j) + (*i_temp).getFrame()[1]->getJacobianOfTranslation(j).cols()) - 1;
      }
      else if ((**i).getType() == "Contact") {  // for contour
//        LinkPartner1Bottom = (**i).getlaInd();
//        LinkPartner2Upper = (**i).getlaInd() + (**i).getlaSize() - 1;
      // only consider the contour in the bottom part of W matrix

        const size_t NoContacts = (*i_temp).getContour().size();
        for (size_t partner = 0; partner < NoContacts; partner++) {
          int lowerRow = (*i_temp).getContour()[partner]->gethInd(j);
          int upperRow = (*i_temp).getContour()[partner]->gethInd(j) + (*i_temp).getContour()[partner]->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON)
              cs_entry(C, row, col, entry);
          }
        }
//        LinkPartner1B = max((*i_temp).getContour()[0]->gethInd(j), (*i_temp).getContour()[1]->gethInd(j));
//        LinkPartner2U = max((*i_temp).getContour()[0]->gethInd(j) + (*i_temp).getContour()[0]->gethSize(j), (*i_temp).getContour()[1]->gethInd(j) + (*i_temp).getContour()[1]->gethSize(j)) - 1;
      }

      else {
        throw MBSimError("Not implemented!");
      }

//      for (column = LinkPartner1Bottom; column <= LinkPartner2Upper; column++) {
//        for (row = LinkPartner1B; row <= LinkPartner2U; row++) {
//          double entry = W[j](row, column);
//          if (fabs(entry) > EPSILON)
//            cs_entry(C, row, col, entry);
//        }
//      }
    }
  }

  // compress triplet format into Compress column format
//  cs_print(C, 0);
  cs * cs_Wj = cs_triplet(C);
//  cs_print(cs_Wj, 0);

  // free the allocated space
  cs_spfree(C);

  return cs_Wj;

}

cs * Perlchain::compressLLM_LToCsparse(int j) {

  int n, nz = 0;
  cs *LLMcs;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  n = LLM[j].cols();

  // check for maximal non-zero-size
  for (size_t i = 0; i < dynamicsystem.size(); i++) {
    nz += dynamicsystem[i]->getuSize(j) * dynamicsystem[i]->getuSize(j);
  }

  for (size_t i = 0; i < object.size(); i++) {
    nz += object[i]->getuSize(j) * object[i]->getuSize(j);
  }

  // creat matrix
  LLMcs = cs_spalloc(n, n, nz, 1, 1); /* allocate memory for the sparse matrix C in Triplet formate, the last input value is 1!*/
  if (!LLMcs)
    return (cs_done(LLMcs, 0, 0, 0)); /* out of memory */

  // Run loop over all sub systems
  for (int i = 0; i < (int) dynamicsystem.size(); i++) {
    int uInd = dynamicsystem[i]->getuInd(j);
    int uSize = dynamicsystem[i]->getuSize(j);

    for (int row = uInd; row < uInd + uSize; row++) {
      for (int col = row; col >= uInd; col--) {
//      for (int col = 0; col < uInd + uSize; col++) {
        double entry = LLM[j](row, col);
        if (fabs(entry) > EPSILON)
          cs_entry(LLMcs, row, col, entry);
      }
    }
  }

  // Run loop over all objects
  for (int i = 0; i < (int) object.size(); i++) {
    int uInd = object[i]->getuInd(j);
    int uSize = object[i]->getuSize(j);

    for (int row = uInd; row < uInd + uSize; row++) {
      for (int col = row; col >= uInd; col--) {
//      for (int col = 0; col < uInd + uSize; col++) {
        double entry = LLM[j](row, col);
        if (fabs(entry) > EPSILON)
          cs_entry(LLMcs, row, col, entry);
      }
    }
  }

  // compress triplet format into Compress column format
//  cs_print(LLMcs, 0);
  cs * cs_Wj = cs_triplet(LLMcs);
//  cs_print(cs_Wj, 0);

  // free the allocated space
  cs_spfree(LLMcs);

  return cs_Wj;

}

Perlchain::Perlchain(const string &projectName) :
    DynamicSystemSolver(projectName) {

  // acceleration of gravity
  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // input flexible ring
  double l0 = 1.;  // length ring
  double E = 2.5e9;  // E-Modul alu
  double rho = 2.5e3;  // density alu
  int elements = 20;  // number of finite elements
  double b0 = 0.02;  // width
  double A = b0 * b0;  // cross-section area
  double I = 1. / 12. * b0 * b0 * b0 * b0;  // moment inertia

  // input infty-norm balls (cuboids)
  int nBalls = 80;  // number of balls
  double mass = 0.025;  // mass of ball

  // flexible ring
  rod = new FlexibleBody1s21RCM("Rod", false);
  rod->setLength(l0);
  rod->setEModul(E);
  rod->setCrossSectionalArea(A);
  rod->setMomentInertia(I);
  rod->setDensity(rho);
  rod->setFrameOfReference(this->getFrame("I"));
  rod->setNumberElements(elements);
  rod->initRelaxed(M_PI / 2.);
  this->addObject(rod);

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::SpineExtrusion *cuboid = new OpenMBV::SpineExtrusion;
  cuboid->setNumberOfSpinePoints(elements * 4); // resolution of visualisation
  cuboid->setDiffuseColor(1 / 3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  vector<OpenMBV::PolygonPoint*> *rectangle = new vector<OpenMBV::PolygonPoint*>; // clockwise ordering, no doubling for closure
  OpenMBV::PolygonPoint *corner1 = new OpenMBV::PolygonPoint(b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  OpenMBV::PolygonPoint *corner2 = new OpenMBV::PolygonPoint(b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
  OpenMBV::PolygonPoint *corner3 = new OpenMBV::PolygonPoint(-b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner3);
  OpenMBV::PolygonPoint *corner4 = new OpenMBV::PolygonPoint(-b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  // balls
  assert(nBalls > 1);
  double d = 7. * l0 / (8. * nBalls); // thickness
  double b = b0 * 1.5; // height / width

  for (int i = 0; i < nBalls; i++) {
    stringstream name;
    name << "Element_" << i;
    RigidBody *ball = new RigidBody(name.str());
    balls.push_back(ball);
    balls[i]->setFrameOfReference(this->getFrame("I"));
    balls[i]->setFrameForKinematics(balls[i]->getFrame("C"));
    balls[i]->setTranslation(new TranslationAlongAxesXY<VecV>);
    balls[i]->setRotation(new RotationAboutZAxis<VecV>);
    balls[i]->setMass(mass);
    SymMat Theta(3, INIT, 0.);
    Theta(0, 0) = 1. / 6. * mass * b * b;
    Theta(1, 1) = 1. / 12. * mass * (d * d + b * b);
    Theta(2, 2) = 1. / 12. * mass * (d * d + b * b);
    balls[i]->setInertiaTensor(Theta);
    this->addObject(balls[i]);

    Point *pt = new Point("COG");
    balls[i]->addContour(pt);

    Point *tP = new Point("topPoint");
    balls[i]->addFrame(new FixedRelativeFrame("topPoint", d * Vec("[0.5;0;0]") + b * Vec("[0;0.5;0]"), SqrMat(3, EYE), balls[i]->getFrame("C")));
    tP->setFrameOfReference(balls[i]->getFrame("topPoint"));
    balls[i]->addContour(tP);

    Point *bP = new Point("bottomPoint");
    balls[i]->addFrame(new FixedRelativeFrame("bottomPoint", d * Vec("[0.5;0;0]") - b * Vec("[0;0.5;0]"), SqrMat(3, EYE), balls[i]->getFrame("C")));
    bP->setFrameOfReference(balls[i]->getFrame("bottomPoint"));
    balls[i]->addContour(bP);

    Plane *plane = new Plane("Plane");
    SqrMat trafoPlane(3, INIT, 0.);
    trafoPlane(0, 0) = -1.;
    trafoPlane(1, 1) = 1.;
    trafoPlane(2, 2) = -1.;
    balls[i]->addFrame(new FixedRelativeFrame("Plane", -d * Vec("[0.5;0;0]"), trafoPlane, balls[i]->getFrame("C")));
    plane->setFrameOfReference(balls[i]->getFrame("Plane"));
    balls[i]->addContour(plane);

#ifdef HAVE_OPENMBVCPPINTERFACE
    OpenMBV::Cuboid *cube = new OpenMBV::Cuboid;
    cube->setLength(d, b, b);
    cube->setDiffuseColor(0, 1, 1);
    balls[i]->setOpenMBVRigidBody(cube);
#endif
  }

  //Set balls to correct position
  FlexibleBody1s21RCM * rodInfo = new FlexibleBody1s21RCM("InfoRod", false);

  rodInfo->setq0(rod->getq());
  rodInfo->setu0(rod->getu());
  rodInfo->setNumberElements(rod->getNumberElements());
  rodInfo->setLength(rod->getLength());
  rodInfo->setFrameOfReference(rod->getFrameOfReference());

  rodInfo->initInfo();
  rodInfo->updateStateDependentVariables(0.);

  for (unsigned int i = 0; i < balls.size(); i++) {
    Vec q0(3, INIT, 0.);
    double xL = fmod(i * rodInfo->getLength() / balls.size() + rodInfo->getLength() * 0.25, rodInfo->getLength());
    ContourPointData cp;
    cp.getContourParameterType() = ContourPointData::continuum;
    cp.getLagrangeParameterPosition()(0) = xL;

    rodInfo->updateKinematicsForFrame(cp, Frame::position_cosy);
    q0(0) = cp.getFrameOfReference().getPosition()(0);
    q0(1) = cp.getFrameOfReference().getPosition()(1);
    q0(2) = -AIK2Cardan(cp.getFrameOfReference().getOrientation())(2) + M_PI * 0.5;
    balls[i]->setInitialGeneralizedPosition(q0);
  }

  delete rodInfo;

  // inertial ball constraint
  this->addFrame(new FixedRelativeFrame("BearingFrame", l0 / (2 * M_PI) * Vec("[0;1;0]"), SqrMat(3, EYE), this->getFrame("I")));
  Joint *joint = new Joint("BearingJoint");
  joint->setForceDirection(Mat("[1,0;0,1;0,0]"));
  joint->setForceLaw(new BilateralConstraint);
  joint->connect(this->getFrame("BearingFrame"), balls[0]->getFrame("C"));
  this->addLink(joint);

  // constraints balls on flexible band
  for (int i = 0; i < nBalls; i++) {
    Contact *contact = new Contact("Band_" + balls[i]->getName());
    contact->setNormalForceLaw(new BilateralConstraint);
    contact->setNormalImpactLaw(new BilateralImpact);
    contact->connect(balls[i]->getContour("COG"), rod->getContour("Contour1sFlexible"));
    contact->enableOpenMBVContactPoints(0.01);
    this->addLink(contact);
  }

  // inner-ball contacts
  for (int i = 0; i < nBalls; i++) {
    stringstream namet, nameb;
    namet << "ContactTop_" << i;
    nameb << "ContactBot_" << i;
    Contact *ctrt = new Contact(namet.str());
    Contact *ctrb = new Contact(nameb.str());
    ctrt->setNormalForceLaw(new UnilateralConstraint);
    ctrt->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    ctrb->setNormalForceLaw(new UnilateralConstraint);
    ctrb->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    if (i == nBalls - 1) {
      ctrt->connect(balls[0]->getContour("topPoint"), balls[i]->getContour("Plane"));
      ctrb->connect(balls[0]->getContour("bottomPoint"), balls[i]->getContour("Plane"));
    }
    else {
      ctrt->connect(balls[i + 1]->getContour("topPoint"), balls[i]->getContour("Plane"));
      ctrb->connect(balls[i + 1]->getContour("bottomPoint"), balls[i]->getContour("Plane"));
    }
    this->addLink(ctrt);
    this->addLink(ctrb);
  }
}


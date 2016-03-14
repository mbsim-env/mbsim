#include "system.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsimFlexibleBody/contours/contour1s_flexible.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsim/functions/kinematics/kinematics.h"

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
using namespace boost;

void Perlchain::initialize() {
  //start

  DynamicSystemSolver::initialize();

  /*Save the non-smooth links to own sets*/
  setValuedContacts.clear();
  setValuedJoints.clear();
  for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
    if ((**i).getType() == "Joint") {  // for joint
      setValuedJoints.push_back(static_cast<Joint*>(*i));
    }
    else if ((**i).getType() == "Contact") {  // for contour
      setValuedContacts.push_back(static_cast<Contact*>(*i));
    }
    else {
      throw MBSimError("Not implemented!");
    }
  }
}

void Perlchain::updateG(double t, int j) {
  const int nDofs = getLLM(t,j).cols();
  const int nLa = getW(t,j).cols();
  if (0) {
    DynamicSystemSolver::updateG(t, j);
  }
  else if (1) {
    cs *cs_L_LLM, *cs_Wj;

    int n = LLM[j].cols();

//      cs_L_LLM = compressLLM_LToCsparse_direct(t, j); // compress the the lower triangular part of LLM into compressed column by constructing the three arrays directly
//      cs_Wj = compressWToCsparse_direct(j); // compress the W into compressed column by constructing the three arrays directly
    cs_L_LLM = compressLLM_LToCsparse(t, j); // transform into the triplet form first and then using cs_triplet (const cs *T) to compress
    cs_Wj = compressWToCsparse(t, j); // transform into the triplet form first and then using cs_triplet (const cs *T) to compress

    int * xi = (int *) cs_malloc(2 * nDofs, sizeof(int));
    double * yy = (double*) calloc(nLa * nDofs, sizeof(double));

    // solve Ly=w
    for (int k = 0; k < nLa; k++) {
      cs_splsolve(cs_L_LLM, cs_Wj, k, xi, yy + k * nDofs, 0); // yele + ylda * k is the pointer to the kth column of y.ele todo: is it acceptable by accessing the private data of y?
    }

    int nzEles = 0;
    SymMatV Gsym(nLa, NONINIT);
    for (int i = 0; i < nLa; i++) {
      for (int j = i; j < nLa; j++) {
        double val = yy[i * nDofs + 0] * yy[j * nDofs + 0];
        for (int k = 1; k < nDofs; k++)
          val += yy[i * nDofs + k] * yy[j * nDofs + k];
        Gsym(i, j) = val;
        if (val > macheps())
          nzEles++;
//          G(i, j) = val;
      }
    }
    nzEles = nzEles * 2 - nLa; //Remark: diagonal is always non-zero!

//todo: free the allocated memory by the cSparse
    cs_spfree(cs_Wj);
    cs_spfree(cs_L_LLM);
    free(xi);
    free(yy);

    if (checkGSize)
      ;  // Gs.resize();
    else if (Gs.cols() != Gsym.size()) {
      static double facSizeGs = 1;
      if (Gsym.size() > limitGSize && fabs(facSizeGs - 1) < epsroot())
        facSizeGs = double(nzEles) / double(Gsym.size() * Gsym.size()) * 1.5;
      Gs.resize(Gsym.size(), int(Gsym.size() * Gsym.size() * facSizeGs));
    }
    Gs << Gsym;
    updG = false;
  }
  else {
    StopWatch sw;
    sw.start();
    bool compare = false;

    cs *cs_L_LLM, *cs_Wj;

    int directCompress = 0;
    // Remark: The compression process is not the one that takes time!
    if (directCompress == 1) {
      cs_L_LLM = compressLLM_LToCsparse_direct(t, j); // compress the the lower triangular part of LLM into compressed column by constructing the three arrays directly
      cs_Wj = compressWToCsparse_direct(j); // compress the W into compressed column by constructing the three arrays directly
    }
    else {
      cs_L_LLM = compressLLM_LToCsparse(t, j); // transform into the triplet form first and then using cs_triplet (const cs *T) to compress
      cs_Wj = compressWToCsparse(j); // transform into the triplet form first and then using cs_triplet (const cs *T) to compress
    }

    int * xi = (int *) cs_malloc(2 * nDofs, sizeof(int));
    Mat y(nDofs, nLa, INIT);
    double * yele = y.operator ()();
    int ylda = y.ldim();

// solve Ly=w

    MatV yV(W[j].cols(), nDofs, INIT);
    double * yeleV = yV.operator ()();
    int yldaV = yV.cols();

    double yy[nDofs * nLa];
    memset(yy, 0, sizeof(yy));

    // solve Ly=w
    for (int k = 0; k < W[j].cols(); k++) {
      cs_splsolve(cs_L_LLM, cs_Wj, k, xi, yele + nDofs * k, 0); // yele + ylda * k is the pointer to the kth column of y.ele todo: is it acceptable by accessing the private data of y?
      cs_splsolve(cs_L_LLM, cs_Wj, k, xi, yeleV + nDofs * k, 0); // yele + ylda * k is the pointer to the kth column of y.ele todo: is it acceptable by accessing the private data of y?
      cs_splsolve(cs_L_LLM, cs_Wj, k, xi, yy + k * nDofs, 0); // yele + ylda * k is the pointer to the kth column of y.ele todo: is it acceptable by accessing the private data of y?
      for (int j = 0; j < nDofs; j++) {
        cout << *(yele + k * nDofs + j) << "\t";
      }
      cout << endl;
      for (int j = 0; j < nDofs; j++) {
        cout << *(yeleV + k * nDofs + j) << "\t";
      }
      cout << endl;
      for (int j = 0; j < nDofs; j++) {
        cout << *(yy + k * nDofs + j) << "\t";
      }
      cout << "------------------------" << endl << endl;
    }

    cout << nrm1(Mat(y - yV.T())) << endl;

    SqrMat GV2(nLa, NONINIT);

    for (int i = 0; i < yV.rows(); i++) {
      VecV temp = trans(yV.row(i)); // temp can be kept in the fast memmory part
      for (int j = i; j < yV.rows(); j++) {
        double val = yV.row(j) * temp;
        GV2(i, j) = val;
        GV2(j, i) = val;
      }
    }

    G.resize(nLa, NONINIT);
    for (int i = 0; i < nLa; i++) {
      for (int j = i; j < nLa; j++) {
        double val = yy[i * nDofs + 0] * yy[j * nDofs + 0];
        for (int k = 1; k < nDofs; k++)
          val += yy[i * nDofs + k] * yy[j * nDofs + k];
        G(i, j) = val;
        G(j, i) = val;
      }
    }

//    for (int i = 0; i < nLa; i++) {
//      for (int j = 0; j < nDofs; j++) {
//        cout << yy[i * nDofs + j] - y(j,i) << ",";
//      }
//      cout << ";" << endl;
//    }
    // the matlab test shows that with full matrix format is faster than the compressed format, this is because that y is stored in compressed-column format and it is hard to access the the column of y'.
    // if G is symmetric type, only the elements in the lower triangular is stored in column wise.
    // if it is accessed in row wise order in the upper triangular part, is will be re-mapped to the lower part,
    // so ele can be still one by one accessed in the order how they stored in memory.

//  calculate G
    SqrMat GV(y.cols(), NONINIT);

    for (int i = 0; i < y.cols(); i++) {
      Vec temp = y.col(i); // temp can be kept in the fast memmory part
      for (int j = i; j < y.cols(); j++) {
        double val = scalarProduct(temp, y.col(j));
        GV(i, j) = val;
        GV(j, i) = val;
      }
    }

    cout << nrm1(G - GV2) << endl;

    if (compare) {
      double t_cs = sw.stop(true);
      sw.start();
      SqrMat Greference(W[j].T() * slvLLFac(LLM[j], getV(t,j)));
      double t_ref = sw.stop(true);
      cout << "t_cs = " << t_cs << " s   | t_ref =" << t_ref << "s   | dt = " << (t_ref - t_cs) / t_ref * 100 << " %" << endl;

      if (0) {
        cout << nrm1(W[j] - cs2Mat(cs_Wj)) << endl;
        cout << W[j] - cs2Mat(cs_Wj) << endl;
        cout << nrm1(LLM[j] - cs2Mat(cs_L_LLM)) << endl;

        ofstream Ofile("W_reference.txt");
        Ofile << W[j];
        Ofile.close();
        Ofile.open("W_cs.txt");
        Ofile << cs2Mat(cs_Wj);
        Ofile.close();
        Ofile.open("W_diff.txt");
        Ofile << W[j] - cs2Mat(cs_Wj);
        Ofile.close();

        Ofile.open("LLM_reference.txt");
        Ofile << LLM[j];
        Ofile.close();
        Ofile.open("LLM_cs.txt");
        Ofile << cs2Mat(cs_L_LLM);
        Ofile.close();
        Ofile.open("LLM_diff.txt");
        Ofile << LLM[j] - cs2Mat(cs_L_LLM);
        Ofile.close();

        ofstream ycsOfile("y_cs.txt");
        ycsOfile << yV;
        ycsOfile.close();

        ofstream GOfile("G_cs.txt");
        GOfile << G;
        GOfile.close();
        ofstream GOreffile("G_reference.txt");
        GOreffile << Greference;
        GOreffile.close();
        SqrMat Gcmp(G - Greference);
        ofstream GOcmpfile("G_diff.txt");
        GOcmpfile << Gcmp;
        GOcmpfile.close();
        cout << nrm1(Gcmp) << endl;
      }
    }

    //todo: free the allocated memory by the cSparse
    cs_spfree(cs_Wj);
    cs_spfree(cs_L_LLM);
    free(xi);

    // compress G into Gs which uses the fmatvec compressed-row format
    if (checkGSize)
      ;  // Gs.resize();
    else if (Gs.cols() != G.size()) {
      static double facSizeGs = 1;
      if (G.size() > limitGSize && fabs(facSizeGs - 1) < epsroot())
        facSizeGs = double(countElements(G)) / double(G.size() * G.size()) * 1.5;
      Gs.resize(G.size(), int(G.size() * G.size() * facSizeGs));
    }
    Gs << G;
    updG = false;
  }
}

cs * Perlchain::compressWToCsparse(double t, int j) {

  int m, n, nzMax; // row, column, nz;//, I1;
  cs *C;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  m = W[j].rows();
  n = W[j].cols();
  nzMax = 5 * m; // todo: find a method to determine the value of nz

  C = cs_spalloc(m, n, nzMax, 1, 1); /* allocate memory for the sparse matrix C in Triplet formate, the last input value is 1!*/
  if (!C)
    return (cs_done(C, 0, 0, 0)); /* out of memory */

  // convert the bottom part of W matrix into cs triplet format
  for (std::vector<Joint*>::iterator it = setValuedJoints.begin(); it != setValuedJoints.end(); ++it) {
    Joint* i_temp = *it;
    for (int col = i_temp->getlaInd(); col < i_temp->getlaInd() + i_temp->getlaSize(); col++) {
      for (size_t partner = 0; partner < 2; partner++) {
        int lowerRow = (*i_temp).getFrame(partner)->gethInd(j);
        int upperRow = (*i_temp).getFrame(partner)->gethInd(j) + (*i_temp).getFrame(partner)->gethSize(j);

        for (int row = lowerRow; row < upperRow; row++) {
          double entry = W[j](row, col);
          if (fabs(entry) > EPSILON)
            cs_entry(C, row, col, entry);
        }
      }
    }
  }

  if (0) {
    //OLD VERSION
    for (std::vector<Contact*>::iterator it = setValuedContacts.begin(); it != setValuedContacts.end(); ++it) {
      Contact* i_temp = *it;
      for (int col = i_temp->getlaInd(); col < i_temp->getlaInd() + i_temp->getlaSize(); col++) {
        for (size_t partner = 0; partner < 2; partner++) {
          int lowerRow = (*i_temp).getContour(partner)->gethInd(j);
          int upperRow = (*i_temp).getContour(partner)->gethInd(j) + (*i_temp).getContour(partner)->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON)
              cs_entry(C, row, col, entry);
          }
        }
      }
    }
  }
  else {
    for (std::vector<Contact*>::iterator it = setValuedContacts.begin(); it != setValuedContacts.end(); ++it) {
      Contact * cnt = *it;
      for (int col = cnt->getlaInd(); col < cnt->getlaInd() + cnt->getlaSize(); col++) {
        for (size_t partner = 0; partner < 2; partner++) {
          int lowerRow = cnt->getContour(partner)->gethInd(j);
          int upperRow = cnt->getContour(partner)->gethInd(j) + cnt->getContour(partner)->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON)
              cs_entry(C, row, col, entry);
          }
        }
      }
    }
  }

// compress triplet format into Compress column format
  cs * cs_Wj = cs_triplet(C);

// free the allocated space
  cs_spfree(C);

  return cs_Wj;
}

cs * Perlchain::compressLLM_LToCsparse(double t, int j) {

  int n, nz;
  cs *LLM_L_csTriplet;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  n = LLM[j].cols();

// check for maximal non-zero-size: right now there are three different ways, when compare the time, the percentage is not reliable for comparing because the ref_time varies a little.

  int nzMethod = 3;

  if (nzMethod == 1) {
    //this method is suitable for a dense LLM, eg, when there are not so many contacs.
    // for sparse LLM, it may allocates about 32 times larger memory than needed, and has to calculate the nz at every time
    // Todo: change the size to 1/2 * uSize * uSize ?   only the lower triangular part of LLM is need to be stored.
    for (size_t i = 0; i < dynamicsystem.size(); i++) {
      nz += dynamicsystem[i]->getuSize(j) * dynamicsystem[i]->getuSize(j);
    }

    for (size_t i = 0; i < object.size(); i++) {
      nz += object[i]->getuSize(j) * object[i]->getuSize(j);
    }
  }
  else if (nzMethod == 2) {
    // this one gives appropriate memory size as needed. nz is only calculate once, but has branches.
    //this one can be still simplified, is j always equto 0
    static int nz0, nz1;
    if (t < 2e-6) { // 2e-6 should be timestep size // todo: find a better way to do this  if(nz0 == 0 || nz1 == 0)?
      if (j == 0)
        nz0 = countElementsLT(LLM[j]) + 50;
      else if (j == 1)
        nz1 = countElementsLT(LLM[j]) + 50;
    }
    nz = (j == 0) ? nz0 : nz1;
    //  cout << "j = " << j << endl; // j is always 0 ??
    //  cout << "nz = " << nz << endl;
  }
  else if (nzMethod == 3) {
    // this one is the simplest one, can avoid extend the memory in most case and does not waste too much memory
    // this one is robust only when construct the LLM by cs_entry()
    // cs_entry will dynamic extend (double) the memory size by realloc(), which will consider whether there are still enough space after the original position.
    nz = 7 * n;
  }
// creat matrix
  LLM_L_csTriplet = cs_spalloc(n, n, nz, 1, 1); /* allocate memory for the sparse matrix C in Triplet format, the last input value is 1!*/
  if (!LLM_L_csTriplet)
    return (cs_done(LLM_L_csTriplet, 0, 0, 0)); /* out of memory */

// Run loop over all sub systems
  for (int i = 0; i < (int) dynamicsystem.size(); i++) {
    int uInd = dynamicsystem[i]->getuInd(j);
    int uSize = dynamicsystem[i]->getuSize(j);

    for (int row = uInd; row < uInd + uSize; row++) {
      for (int col = row; col >= uInd; col--) {
//      for (int col = 0; col < uInd + uSize; col++) {
        double entry = LLM[j](row, col);
        if (fabs(entry) > EPSILON)
          cs_entry(LLM_L_csTriplet, row, col, entry);
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
          cs_entry(LLM_L_csTriplet, row, col, entry);
      }
    }
  }

// compress triplet format into Compress column format
  cs * LLM_L_cs = cs_triplet(LLM_L_csTriplet);

// free the allocated space
  cs_spfree(LLM_L_csTriplet);

  return LLM_L_cs;

}

cs * Perlchain::compressWToCsparse_direct(double t, int j) {

  int m, n, nz, *Cp, *Ci, counter;
  ;
  double *Cx;
  ;
  cs *C;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  m = W[j].rows();
  n = W[j].cols();
  nz = 7 * m; // todo: find a method to determine the value of nz

  C = cs_spalloc(m, n, nz, 1, 0); /* allocate memory for the sparse matrix C in compressed column format, the last input value is 0!*/
  if (!C)
    return (cs_done(C, 0, 0, 0)); /* out of memory */

  Cp = C->p;
  Ci = C->i;
  Cx = C->x;

  counter = 0;
  for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i) {
    for (int col = (**i).getlaInd(); col < (**i).getlaInd() + (**i).getlaSize(); col++) {
      Cp[col] = counter;
      if ((**i).getType() == "Joint") {  // for joint
        Joint * i_temp = dynamic_cast<Joint*>(*i);
        for (size_t partner = 0; partner < 2; partner++) {
          int lowerRow = (*i_temp).getFrame(partner)->gethInd(j);
          int upperRow = (*i_temp).getFrame(partner)->gethInd(j) + (*i_temp).getFrame(partner)->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON) {
              Ci[counter] = row;
              Cx[counter] = entry;
              counter++;
            }
          }
        }
      }
      else if ((**i).getType() == "Contact") {  // for contact
        Contact * i_temp = dynamic_cast<Contact*>(*i);
        for (size_t partner = 0; partner < 2; partner++) {
          int lowerRow = (*i_temp).getContour(partner)->gethInd(j);
          int upperRow = (*i_temp).getContour(partner)->gethInd(j) + (*i_temp).getContour(partner)->gethSize(j);

          for (int row = lowerRow; row < upperRow; row++) {
            double entry = W[j](row, col);
            if (fabs(entry) > EPSILON) {
              Ci[counter] = row;
              Cx[counter] = entry;
              counter++;
            }
          }
        }
      }

      else {
        THROW_MBSIMERROR("Not implemented!");
      }
    }
  }

  Cp[n] = counter;

// free the allocated space
  return C;

}

cs * Perlchain::compressLLM_LToCsparse_direct(double t, int j) {

  int n, nzMax, *Cp, *Ci, counter;
  ;
  double *Cx;
  cs *LLM_L_cs;
  double EPSILON = 1e-17; /*todo: a better epsilion? */

  n = LLM[j].cols();

  int nzMethod = 3; //todo: for the dense LLM, the memory may not enough for the thrid method.

  if (nzMethod == 1) {
    //this method is suitable for a dense LLM, eg, when there are not so many contacs.
    // for sparse LLM, it may allocates about 32 times larger memory than needed, and has to calculate the nz at every time
    // Todo: change the size to 1/2 * uSize * uSize ?   only the lower triangular part of LLM is need to be stored.
    for (size_t i = 0; i < dynamicsystem.size(); i++) {
      nzMax += dynamicsystem[i]->getuSize(j) * dynamicsystem[i]->getuSize(j);
    }

    for (size_t i = 0; i < object.size(); i++) {
      nzMax += object[i]->getuSize(j) * object[i]->getuSize(j);
    }
  }
  else if (nzMethod == 2) {
    // this one gives appropriate memory size as needed. nz is only calculate once, but has branches.
    //this one can be still simplified, is j always equto 0
    static int nz0, nz1;
    if (t < 2e-6) { // 2e-6 should be timestep size // todo: find a better way to do this  if(nz0 == 0 || nz1 == 0)?
      if (j == 0)
        nz0 = countElementsLT(LLM[j]) + 50;
      else if (j == 1)
        nz1 = countElementsLT(LLM[j]) + 50;
    }
    nzMax = (j == 0) ? nz0 : nz1;
  }
  else if (nzMethod == 3) {
    //todo: for the dense LLM, the memory may not enough.
    nzMax = 7 * n;
  }

// creat matrix
  LLM_L_cs = cs_spalloc(n, n, nzMax, 1, 0); /* allocate memory for the sparse matrix C in compressed column format, the last input value is 0 !*/
  if (!LLM_L_cs)
    return (cs_done(LLM_L_cs, 0, 0, 0)); /* out of memory */

  Cp = LLM_L_cs->p;
  Ci = LLM_L_cs->i;
  Cx = LLM_L_cs->x;

//create the three arrays
  counter = 0;
// Run loop over all sub systems
  for (int i = 0; i < (int) dynamicsystem.size(); i++) {
    int uInd = dynamicsystem[i]->getuInd(j);
    int uSize = dynamicsystem[i]->getuSize(j);

    for (int col = uInd; col < uInd + uSize; col++) {
      Cp[col] = counter;
      for (int row = col; row < uInd + uSize; row++) {
        double entry = LLM[j](row, col);
//        if (fabs(entry) > EPSILON) {
        Ci[counter] = row;
        Cx[counter] = entry;
        counter++;
//        }
      }
    }

  }

// Run loop over all objects
  for (int i = 0; i < (int) object.size(); i++) {
    int uInd = object[i]->getuInd(j);
    int uSize = object[i]->getuSize(j);

    for (int col = uInd; col < uInd + uSize; col++) {
      Cp[col] = counter;
      for (int row = col; row < uInd + uSize; row++) {
        double entry = LLM[j](row, col);
//        if (fabs(entry) > EPSILON) {
        Ci[counter] = row;
        Cx[counter] = entry;
        counter++;
//        }
      }
    }

  }

  Cp[n] = counter;

  return LLM_L_cs;
}

Perlchain::Perlchain(const string &projectName) :
    DynamicSystemSolver(projectName) {



// acceleration of gravity
  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

// input flexible ring
  double l0 = 1.; // length ring
  double E = 2.5e9; // E-Modul alu
  double rho = 2.5e3; // density alu
  int elements = 20; // number of finite elements
  double b0 = 0.02; // width
  double A = b0 * b0; // cross-section area
  double I = 1. / 12. * b0 * b0 * b0 * b0; // moment inertia

// input infty-norm balls (cuboids)
  int nBalls = 80; // number of balls
  double mass = 0.025; // mass of ball

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
  boost::shared_ptr<OpenMBV::SpineExtrusion> cuboid = OpenMBV::ObjectFactory::create<OpenMBV::SpineExtrusion>();
  cuboid->setNumberOfSpinePoints(elements * 4); // resolution of visualisation
  cuboid->setDiffuseColor(1 / 3.0, 1, 1); // color in (minimalColorValue, maximalColorValue)
  cuboid->setScaleFactor(1.); // orthotropic scaling of cross section
  shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > rectangle = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >(); // clockwise ordering, no doubling for closure
  shared_ptr<OpenMBV::PolygonPoint> corner1 = OpenMBV::PolygonPoint::create(b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner1);
  shared_ptr<OpenMBV::PolygonPoint> corner2 = OpenMBV::PolygonPoint::create(b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner2);
  shared_ptr<OpenMBV::PolygonPoint> corner3 = OpenMBV::PolygonPoint::create(-b0 * 0.5, -b0 * 0.5, 1);
  rectangle->push_back(corner3);
  shared_ptr<OpenMBV::PolygonPoint> corner4 = OpenMBV::PolygonPoint::create(-b0 * 0.5, b0 * 0.5, 1);
  rectangle->push_back(corner4);

  cuboid->setContour(rectangle);
  rod->setOpenMBVSpineExtrusion(cuboid);
#endif

  Contour1sFlexible *neutral = new Contour1sFlexible("Neutral");
  rod->addContour(neutral);

  FlexibleBand *contour1sFlexible = new FlexibleBand("Contour1sFlexible");
  Vec nodes(elements+1);
  for(int i=0;i<=elements;i++) nodes(i) = i*l0/elements;
  contour1sFlexible->setNodes(nodes);
  contour1sFlexible->setWidth(0.1);
  contour1sFlexible->setContourOfReference(neutral);
  rod->addContour(contour1sFlexible);

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
    boost::shared_ptr<OpenMBV::Cuboid> cube = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
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

  for (unsigned int i = 0; i < balls.size(); i++) {
    Vec q0(3, INIT, 0.);
    double xL = fmod(i * rodInfo->getLength() / balls.size() + rodInfo->getLength() * 0.25, rodInfo->getLength());

    Vec3 r = rodInfo->getPosition(0,xL);
    q0(0) = r(0);
    q0(1) = r(1);

    SqrMat3 A = rodInfo->getOrientation(0,xL);
     cout <<  fmod(AIK2Cardan(A)(2)+M_PI,2*M_PI) << endl;
    q0(2) = fmod(AIK2Cardan(A)(2)+M_PI,2*M_PI);
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

  for(int i=0;i<nBalls;i++) {
    Contact *contact = new Contact("Band_"+balls[i]->getName());
    contact->setNormalForceLaw(new BilateralConstraint);
    contact->setNormalImpactLaw(new BilateralImpact);
    contact->connect(balls[i]->getContour("COG"),rod->getContour("Contour1sFlexible"));
    contact->enableOpenMBVContactPoints(0.01);
    contact->setSearchAllContactPoints(true);
    this->addLink(contact);
  }

  // inner-ball contacts
  for(int i=0;i<nBalls;i++) {
    stringstream namet,nameb;
    namet << "ContactTop_" << i;
    nameb << "ContactBot_" << i;
    Contact *ctrt = new Contact(namet.str());
    Contact *ctrb = new Contact(nameb.str());
    ctrt->setNormalForceLaw(new UnilateralConstraint);
    ctrt->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    ctrb->setNormalForceLaw(new UnilateralConstraint);
    ctrb->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    if(i==nBalls-1) {
      ctrt->connect(balls[0]->getContour("topPoint"),balls[i]->getContour("Plane"));
      ctrb->connect(balls[0]->getContour("bottomPoint"),balls[i]->getContour("Plane"));
    }
    else {
      ctrt->connect(balls[i+1]->getContour("topPoint"),balls[i]->getContour("Plane"));
      ctrb->connect(balls[i+1]->getContour("bottomPoint"),balls[i]->getContour("Plane"));
    }
    this->addLink(ctrt);
    this->addLink(ctrb);
  }
// constraints balls on flexible band
//  Contact *contact = new Contact("Band_Balls"); // + balls[i]->getName());
//  contact->setNormalForceLaw(new BilateralConstraint);
//  contact->setNormalImpactLaw(new BilateralImpact);
//  contact->enableOpenMBVContactPoints(0.01);
//  contact->setSearchAllContactPoints(true);
//  this->addLink(contact);
//  for (int i = 0; i < nBalls; i++) {
//    contact->connect(balls[i]->getContour("COG"), rod->getContour("Contour1sFlexible"));
//  }
//
//// inner-ball contacts
//  stringstream namet, nameb;
//  namet << "ContactTop";
//  nameb << "ContactBot";
//  Contact *ctrt = new Contact(namet.str());
//  Contact *ctrb = new Contact(nameb.str());
//  ctrt->setNormalForceLaw(new UnilateralConstraint);
//  ctrt->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
//  ctrb->setNormalForceLaw(new UnilateralConstraint);
//  ctrb->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
//  this->addLink(ctrt);
//  this->addLink(ctrb);
//  for (int i = 0; i < nBalls; i++) {
//    if (i == nBalls - 1) {
//      ctrt->connect(balls[0]->getContour("topPoint"), balls[i]->getContour("Plane"));
//      ctrb->connect(balls[0]->getContour("bottomPoint"), balls[i]->getContour("Plane"));
//    }
//    else {
//      ctrt->connect(balls[i + 1]->getContour("topPoint"), balls[i]->getContour("Plane"));
//      ctrb->connect(balls[i + 1]->getContour("bottomPoint"), balls[i]->getContour("Plane"));
//    }
//  }
}


// Isothermal hydrodynamic (HD) contact in journal bearing
// Michael Hofer, 13.04.2015

#include <fmatvec/fmatvec.h>

#include "ehd_pressure_element.h"
#include "ehd_mesh.h"
#include "journal_bearing.h"
#include "lubricant.h"

#include <mbsim/utils/utils.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimEHD;

void TestLubricants(Lubricant & lubT1, Lubricant & lubT2) {
  // Synthetic paraffnic oil (lot 3) at two different temperatures, see
  // Hamrock and Jones
  // Michael Hofer, 18.05.2015

  // --------------------------------------------------------------------
  // T1 = 38°C
  // --------------------------------------------------------------------
  double eta0T1 = 0.414;     // in Ns/m^2
  double rho0T1 = 839;       // in kg/m^3
  double ZT1 = 0.43;
  double alphaT1 = 1.77e-8;  // in m^2/N

  lubT1 = Lubricant(eta0T1, rho0T1, ZT1, false, Lubricant::Roelands, Lubricant::DowsonHigginson);
  //lubT1{2} = Lubricant(eta0T1, rho0T1, ZT1, false "Barus", "DowsonHigginson");
  //lubT1{3} = Lubricant(eta0T1, rho0T1, 0, false, "constant", "constant");
  //lubT1{4} = Lubricant(eta0T1, rho0T1, ZT1, false "Roelands", "constant");
  //lubT1{5} = Lubricant(eta0T1, rho0T1, alphaT1, true, "Barus", "DowsonHigginson");

  // --------------------------------------------------------------------
  // T2 = 149°C
  // --------------------------------------------------------------------

  double eta0T2 = 0.0109;    // in Ns/m^2
  double rho0T2 = 778;       // in kg/m^3
  double ZT2 = 0.39;
  double alphaT2 = 1.09e-8;  // in m^2/N

  //lubT2 = Lubricant(eta0T2, rho0T2, ZT2, false, "Roelands", "DowsonHigginson");
  //lubT2 = Lubricant(eta0T2, rho0T2, ZT2, false "Barus", "DowsonHigginson");
  lubT2 = Lubricant(eta0T2, rho0T2, 0, false, Lubricant::constVisc, Lubricant::constDen);
  //lubT2 = Lubricant(eta0T2, rho0T2, ZT2, false, "Roelands", "constant");
  //lubT2 = Lubricant(eta0T2, rho0T2, alphaT2, true, "Barus", "DowsonHigginson");
}

int SimJournalBearing() {
// ------------------------------------------------------------------------
// System and lubricant
// ------------------------------------------------------------------------

// Set up system for journal bearing with constant angular velocity
  JournalBearing sys;
  sys.setomega1(500);
  sys.setFr(10e3);

// Set mass matrix of multi body system (body 1 is the only body with DOFs)
  SqrMat3 M = sys.getM1();

// Get lubricants at two different temperatures

// Define lubricants for simulation
  Lubricant lubT1, // 'Roelands', 'DowsonHigginson'
      lubT2; // 'constant', 'constant'
  TestLubricants(lubT1, lubT2);

// ------------------------------------------------------------------------
// Spatial discretization
// ------------------------------------------------------------------------

// Create object of element used for discretization
  EHDPressureElement ele("quad9", 4);
  ele.setLubricant(lubT2);

// Create computational mesh (half fluid domain)
  RowVec2 yb;
  yb(0) = 0;
  yb(1) = 2 * M_PI * sys.getR2();
  RowVec2 zb;
  zb(0) = -1 * sys.getL() / 2;
  zb(1) = 1 * sys.getL() / 2;
  MatVx2 xb(2);
  xb.set(0, yb);
  xb.set(1, zb);

  EHDMesh msh(ele, xb, VecInt("[20; 3]"));

  msh.Boundary(EHDMesh::dbc, EHDMesh::x2m);    // z = -L / 2
  msh.Boundary(EHDMesh::per1, EHDMesh::x1m);   // y = 0
  msh.Boundary(EHDMesh::per2, EHDMesh::x1p);   // y = 2 * pi * R2
  msh.FinishMesh();

  VecV D(msh.getndof());
  msh.PressureAssembly(D, sys);

  VecV R = msh.getR();
  SqrMatV KT = msh.getKT();

  VecV P(msh.getndof());
  VecV Pfree(msh.getnfree());
  VecInt fdofs(msh.getfreedofs());
  VecVI ipiv(subMat(KT, fdofs, -1).rows());
  SqrMatV JLU = facLU(subMat(KT, fdofs, -1), ipiv);
  Pfree = slvLUFac(JLU, subVec(R, fdofs, -1), ipiv);

  for (int i = 0; i < msh.getnfree(); i++) {
    P(fdofs(i) - 1) = Pfree(i);
  }

//  cout << R << endl;
//  cout << KT.col(1) << endl;
  cout << P << endl;

//TODO: Testing (compare with matlab stuff) -->>  how to do?
  ifstream myfile;
  myfile.open("msh_pos_matlab.txt");
  std::string str((std::istreambuf_iterator<char>(myfile)), std::istreambuf_iterator<char>());
  myfile.close();

  string final("[" + str + "]");
  cout << final.c_str() << endl;
  VecV matlCmp(final.c_str());
  cout << "Difference in pos: " << nrm2(matlCmp - msh.getpos()) << endl;

  myfile.open("msh_locx_matlab.txt");
  str = string((std::istreambuf_iterator<char>(myfile)), std::istreambuf_iterator<char>());
  myfile.close();

  final = string("[" + str + "]");
  cout << final.c_str() << endl;
  MatVI matlCmpMatVI(final.c_str());
  cout << "Difference in locX: " << matlCmpMatVI - msh.getlocX() << endl;

  myfile.open("msh_locd_matlab.txt");
  str = string((std::istreambuf_iterator<char>(myfile)), std::istreambuf_iterator<char>());
  myfile.close();

  final = string("[" + str + "]");
  cout << final.c_str() << endl;
  MatVI matlCmpMatlocD(final.c_str());
  cout << "Difference in locD: " << matlCmpMatlocD - msh.getlocD() << endl;

  return 0;

}

/*
 // ------------------------------------------------------------------------
 // Time integration with Newmark scheme using a quasi-Newton method
 // ------------------------------------------------------------------------

 // Set time interval [t0, tE] and constant time step size
 t0 = 0;
 tE = t0 + 2 * pi / sys.omega1; // one revolution
 deltat = 5e-5;

 // Test equlibrium
 // deltat = 1e-5;
 // sys.Fr = 30e3;
 // sys.IxS1 = [0.371; -0.742] * sys.h0; // Roelands, Dowson and Higginson
 // sys.IxS1 = [0.392; -0.771] * sys.h0; // constant, constant

 // Set inital state vector and nodal solution vector
 z0 = [sys.IxS1; sys.phi1; sys.IuS1; sys.omega1];
 D0 = zeros(msh.ndof, 1);

 // Pressure dependent fluid parameters
 saveSol = 0;
 loadSol = 1;
 if ~loadSol
 tic;
 [Tlub1, Zlub1, Ilub1, Pmaxlub1] = NewmarkQuasiNewton(...
 sys, msh, lub1, t0, tE, deltat, z0, D0, M, 1/4, 1/2);
 toc
 if saveSol
 save([dirSave, "Tlub1"], "Tlub1");
 save([dirSave, "Zlub1"], "Zlub1");
 save([dirSave, "Ilub1"], "Ilub1");
 save([dirSave, "Pmaxlub1"], "Pmaxlub1");
 end
 else
 load([dirSave, "Tlub1"], "Tlub1");
 load([dirSave, "Zlub1"], "Zlub1");
 load([dirSave, "Ilub1"], "Ilub1");
 load([dirSave, "Pmaxlub1"], "Pmaxlub1");
 end

 // Constant fluid parameters
 saveSol = 0;
 loadSol = 1;
 if ~loadSol
 tic;
 [Tlub3, Zlub3, Ilub3, Pmaxlub3] = NewmarkQuasiNewton(...
 sys, msh, lub3, t0, tE, deltat, z0, D0, M, 1/4, 1/2);
 toc
 if saveSol
 save([dirSave, "Tlub3"], "Tlub3");
 save([dirSave, "Zlub3"], "Zlub3");
 save([dirSave, "Ilub3"], "Ilub3");
 save([dirSave, "Pmaxlub3"], "Pmaxlub3");
 end
 else
 load([dirSave, "Tlub3"], "Tlub3");
 load([dirSave, "Zlub3"], "Zlub3");
 load([dirSave, "Ilub3"], "Ilub3");
 load([dirSave, "Pmaxlub3"], "Pmaxlub3");
 end

 */
/*
 // ------------------------------------------------------------------------
 // Postprocess pressure distribution
 // ------------------------------------------------------------------------

 // Pressure dependent fluid parameters
 saveSol = 0;
 loadSol = 1;
 n = length(Tlub1);
 if ~loadSol
 DTlub1 = cell(n, 1);
 for i = 1:1:n
 // Print progress
 fprintf('Time step //d///d\n", i, n);
 [~, DTlub1{i}] = ForceVector(sys, msh, lub1, Tlub1(i), Zlub1(i, :)', D0);
 end
 if saveSol
 save([dirSave, "DTlub1"], "DTlub1");
 end
 else
 load([dirSave, "DTlub1"], "DTlub1");
 end

 // Constant fluid parameters
 saveSol = 0;
 loadSol = 1;
 n = length(Tlub3);
 if ~loadSol
 DTlub3 = cell(n, 1);
 for i = 1:1:n
 // Print progress
 fprintf("Time step //d///d\n", i, n);
 [~, DTlub3{i}] = ForceVector(sys, msh, lub3, Tlub3(i), Zlub3(i, :)', D0);
 end
 if saveSol
 save([dirSave, "DTlub3"], "DTlub3");
 end
 else
 load([dirSave, "DTlub3"], "DTlub3");
 end


 // ------------------------------------------------------------------------
 // Draw solution
 // ------------------------------------------------------------------------

 // Define options
 colors = Colors();
 s = {"EdgeColor", "w", "Interpreter", "latex"};
 leg = {"$\eta_\mathrm{Roe}$, $\rho_\mathrm{DowHig}$", ...
 "$\eta_\mathrm{konst}$, $\rho_\mathrm{konst}$"};

 // Define units
 unitp = "MPa";
 unitt = "ms";
 unitx = "mm";
 [unitFacp, unitLabelp] = Units(unitp, false);
 [unitFact, unitLabelt] = Units(unitt, false);
 [unitFacx, unitLabelx] = Units(unitx, false);

 labelp = ["$p$ in ", unitLabelp];
 labelpmax = ["$p_\mathrm{max}$ in ", unitLabelp];
 labelt = ["$t$ in ", unitLabelt];
 labelI = "$I_\mathrm{QN}$";

 labelex = "$e_x / h_0$";
 labeley = "$e_y / h_0$";

 draw = "eccentr";
 switch draw
 case "pmax"
 figure;
 plot(Tlub1 * unitFact, Pmaxlub1 * unitFacp, "Color", colors.red);
 hold on;
 plot(Tlub3 * unitFact, Pmaxlub3 * unitFacp, "Color", colors.blue);
 grid on;
 legend(leg, "Location", "southEast", s{:});
 xlabel([labelt, " $\longrightarrow$"], "Interpreter", "latex");
 ylabel([labelpmax, " $\longrightarrow$"], "Interpreter", "latex");

 case "iter"
 figure;
 plot(Tlub1 * unitFact, Ilub1, "Color", colors.red);
 hold on;
 plot(Tlub3 * unitFact, Ilub3, "Color", colors.blue);
 grid on;
 legend(leg, "Location", "southEast", s{:});
 xlabel([labelt, " $\longrightarrow$"], "Interpreter", "latex");
 ylabel([labelI, " $\longrightarrow$"], "Interpreter", "latex");

 case "eccentr"
 figure;
 plot(Zlub1(:, 1) / sys.h0, Zlub1(:, 2) / sys.h0, "-", "Color", colors.red);
 hold on;
 plot(Zlub3(:, 1) / sys.h0, Zlub3(:, 2) / sys.h0, "-", "Color", colors.blue);
 grid on;
 axis image;
 set(gca, "xlim", [-1, 1]);
 set(gca, "ylim", [-1, 1]);
 lh = legend(leg, "Location", "northEast", s{:});
 pos = get(lh, "Position");
 set(lh, "Position", [0.43, 0.3, pos(3), pos(4)]);
 xlabel([labelex, " $\longrightarrow$"], "Interpreter", "latex");
 ylabel([labeley, " $\longrightarrow$"], "Interpreter", "latex");

 case "video"
 CreateVideo(Tlub1, Zlub1, DTlub1, sys, msh, "../../../60_Vortraege/Video");
 end
 */

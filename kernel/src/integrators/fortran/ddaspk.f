C DASPK2.0 from netlib.org
C in MBSim imported source files: ddaspk.f daux.f
C adaptions:
C		DATV   (ddaspk.f) renamed in DATV1 (conflict with opkda1.f)
C		DHELS  (ddaspk.f) commented out    (opkda1.f routine is used instead)
C		DHEQR  (ddaspk.f) commented out    (opkda1.f routine is used instead)
C		DUMSUM (daux.f)	  commented out    (opkda1.f routine is used instead)
C		IXSAV  (daux.f)   commented out    (opkda2.f routine is used instead)
C		XSETUN (daux.f)   commented out    (opkda2.f routine is used instead)
C		XSETF  (daux.f)   commented out    (opkda2.f routine is used instead)
C		XERRWD (daux.f)   commented out    (opkda2.f routine is used instead)
C   all subroutines expect DDASPK and DATV1 are commented out (routines of ddaspkr.f are used instead)
C
C   scaling of variables for error test added
C    - subroutine DCALCVT added
C    - basis of IWORK extended to 50
C    - see end of file for detailed list of modifications
C
C R.Huber 18/07/2008 

      SUBROUTINE DDASPK (RES, NEQ, T, Y, YPRIME, TOUT, INFO, RTOL, ATOL,
     *   IDID, RWORK, LRW, IWORK, LIW, RPAR, IPAR, JAC, PSOL)
C
C***BEGIN PROLOGUE  DDASPK
C***DATE WRITTEN   890101   (YYMMDD)
C***REVISION DATE  910624   (Added HMAX test at 525 in main driver.)
C***REVISION DATE  920929   (CJ in RES call, RES counter fix.)
C***REVISION DATE  921215   (Warnings on poor iteration performance)
C***REVISION DATE  921216   (NRMAX as optional input)
C***REVISION DATE  930315   (Name change: DDINI to DDINIT)
C***REVISION DATE  940822   (Replaced initial condition calculation)
C***REVISION DATE  941101   (Added linesearch in I.C. calculations)
C***REVISION DATE  941220   (Misc. corrections throughout)
C***REVISION DATE  950125   (Added DINVWT routine)
C***REVISION DATE  950714   (Misc. corrections throughout)
C***REVISION DATE  950802   (Default NRMAX = 5, based on tests.)
C***REVISION DATE  950808   (Optional error test added.)
C***REVISION DATE  950814   (Added I.C. constraints and INFO(14))
C***REVISION DATE  950828   (Various minor corrections.)
C***REVISION DATE  951006   (Corrected WT scaling in DFNRMK.)
C***REVISION DATE  951030   (Corrected history update at end of DDASTP.)
C***REVISION DATE  960129   (Corrected RL bug in DLINSD, DLINSK.)
C***REVISION DATE  960301   (Added NONNEG to SAVE statement.)
C***REVISION DATE  000512   (Removed copyright notices.)
C***REVISION DATE  000622   (Corrected LWM value using NCPHI.)
C***REVISION DATE  000628   (Corrected I.C. stopping tests when index = 0.)
C***REVISION DATE  000628   (Fixed alpha test in I.C. calc., Krylov case.)
C***REVISION DATE  000628   (Improved restart in I.C. calc., Krylov case.)
C***REVISION DATE  000628   (Minor corrections throughout.)
C***REVISION DATE  000711   (Fixed Newton convergence test in DNSD, DNSK.) 
C***REVISION DATE  000712   (Fixed tests on TN - TOUT below 420 and 440.)
C
C***REVISION DATE  080724   (YYMMDD) H dependened error test added (R. Huber)
C
C***CATEGORY NO.  I1A2
C***KEYWORDS  DIFFERENTIAL/ALGEBRAIC, BACKWARD DIFFERENTIATION FORMULAS,
C             IMPLICIT DIFFERENTIAL SYSTEMS, KRYLOV ITERATION
C***AUTHORS   Linda R. Petzold, Peter N. Brown, Alan C. Hindmarsh, and
C                  Clement W. Ulrich
C             Center for Computational Sciences & Engineering, L-316
C             Lawrence Livermore National Laboratory
C             P.O. Box 808,
C             Livermore, CA 94551
C***PURPOSE  This code solves a system of differential/algebraic 
C            equations of the form 
C               G(t,y,y') = 0 , 
C            using a combination of Backward Differentiation Formula 
C            (BDF) methods and a choice of two linear system solution 
C            methods: direct (dense or band) or Krylov (iterative).
C            This version is in double precision.
C-----------------------------------------------------------------------
C***DESCRIPTION
C
C *Usage:
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      INTEGER NEQ, INFO(N), IDID, LRW, LIW, IWORK(LIW), IPAR(*)
C      DOUBLE PRECISION T, Y(*), YPRIME(*), TOUT, RTOL(*), ATOL(*),
C         RWORK(LRW), RPAR(*)
C      EXTERNAL  RES, JAC, PSOL
C
C      CALL DDASPK (RES, NEQ, T, Y, YPRIME, TOUT, INFO, RTOL, ATOL,
C     *   IDID, RWORK, LRW, IWORK, LIW, RPAR, IPAR, JAC, PSOL)
C
C  Quantities which may be altered by the code are:
C     T, Y(*), YPRIME(*), INFO(1), RTOL, ATOL, IDID, RWORK(*), IWORK(*)
C
C *Arguments:
C
C  RES:EXT          This is the name of a subroutine which you
C                   provide to define the residual function G(t,y,y')
C                   of the differential/algebraic system.
C
C  NEQ:IN           This is the number of equations in the system.
C
C  T:INOUT          This is the current value of the independent 
C                   variable.
C
C  Y(*):INOUT       This array contains the solution components at T.
C
C  YPRIME(*):INOUT  This array contains the derivatives of the solution
C                   components at T.
C
C  TOUT:IN          This is a point at which a solution is desired.
C
C  INFO(N):IN       This is an integer array used to communicate details
C                   of how the solution is to be carried out, such as
C                   tolerance type, matrix structure, step size and
C                   order limits, and choice of nonlinear system method.
C                   N must be at least 20.
C
C  RTOL,ATOL:INOUT  These quantities represent absolute and relative
C                   error tolerances (on local error) which you provide
C                   to indicate how accurately you wish the solution to
C                   be computed.  You may choose them to be both scalars
C                   or else both arrays of length NEQ.
C
C  IDID:OUT         This integer scalar is an indicator reporting what
C                   the code did.  You must monitor this variable to
C                   decide what action to take next.
C
C  RWORK:WORK       A real work array of length LRW which provides the
C                   code with needed storage space.
C
C  LRW:IN           The length of RWORK.
C
C  IWORK:WORK       An integer work array of length LIW which provides
C                   the code with needed storage space.
C
C  LIW:IN           The length of IWORK.
C
C  RPAR,IPAR:IN     These are real and integer parameter arrays which
C                   you can use for communication between your calling
C                   program and the RES, JAC, and PSOL subroutines.
C
C  JAC:EXT          This is the name of a subroutine which you may
C                   provide (optionally) for calculating Jacobian 
C                   (partial derivative) data involved in solving linear
C                   systems within DDASPK.
C
C  PSOL:EXT         This is the name of a subroutine which you must
C                   provide for solving linear systems if you selected
C                   a Krylov method.  The purpose of PSOL is to solve
C                   linear systems involving a left preconditioner P.
C
C *Overview
C
C  The DDASPK solver uses the backward differentiation formulas of
C  orders one through five to solve a system of the form G(t,y,y') = 0
C  for y = Y and y' = YPRIME.  Values for Y and YPRIME at the initial 
C  time must be given as input.  These values should be consistent, 
C  that is, if T, Y, YPRIME are the given initial values, they should 
C  satisfy G(T,Y,YPRIME) = 0.  However, if consistent values are not
C  known, in many cases you can have DDASPK solve for them -- see INFO(11).
C  (This and other options are described in more detail below.)
C
C  Normally, DDASPK solves the system from T to TOUT.  It is easy to
C  continue the solution to get results at additional TOUT.  This is
C  the interval mode of operation.  Intermediate results can also be
C  obtained easily by specifying INFO(3).
C
C  On each step taken by DDASPK, a sequence of nonlinear algebraic  
C  systems arises.  These are solved by one of two types of
C  methods:
C    * a Newton iteration with a direct method for the linear
C      systems involved (INFO(12) = 0), or
C    * a Newton iteration with a preconditioned Krylov iterative 
C      method for the linear systems involved (INFO(12) = 1).
C
C  The direct method choices are dense and band matrix solvers, 
C  with either a user-supplied or an internal difference quotient 
C  Jacobian matrix, as specified by INFO(5) and INFO(6).
C  In the band case, INFO(6) = 1, you must supply half-bandwidths
C  in IWORK(1) and IWORK(2).
C
C  The Krylov method is the Generalized Minimum Residual (GMRES) 
C  method, in either complete or incomplete form, and with 
C  scaling and preconditioning.  The method is implemented
C  in an algorithm called SPIGMR.  Certain options in the Krylov 
C  method case are specified by INFO(13) and INFO(15).
C
C  If the Krylov method is chosen, you may supply a pair of routines,
C  JAC and PSOL, to apply preconditioning to the linear system.
C  If the system is A*x = b, the matrix is A = dG/dY + CJ*dG/dYPRIME
C  (of order NEQ).  This system can then be preconditioned in the form
C  (P-inverse)*A*x = (P-inverse)*b, with left preconditioner P.
C  (DDASPK does not allow right preconditioning.)
C  Then the Krylov method is applied to this altered, but equivalent,
C  linear system, hopefully with much better performance than without
C  preconditioning.  (In addition, a diagonal scaling matrix based on
C  the tolerances is also introduced into the altered system.)
C
C  The JAC routine evaluates any data needed for solving systems
C  with coefficient matrix P, and PSOL carries out that solution.
C  In any case, in order to improve convergence, you should try to
C  make P approximate the matrix A as much as possible, while keeping
C  the system P*x = b reasonably easy and inexpensive to solve for x,
C  given a vector b.
C
C
C *Description
C
C------INPUT - WHAT TO DO ON THE FIRST CALL TO DDASPK-------------------
C
C
C  The first call of the code is defined to be the start of each new
C  problem.  Read through the descriptions of all the following items,
C  provide sufficient storage space for designated arrays, set
C  appropriate variables for the initialization of the problem, and
C  give information about how you want the problem to be solved.
C
C
C  RES -- Provide a subroutine of the form
C
C             SUBROUTINE RES (T, Y, YPRIME, CJ, DELTA, IRES, RPAR, IPAR)
C
C         to define the system of differential/algebraic
C         equations which is to be solved. For the given values
C         of T, Y and YPRIME, the subroutine should return
C         the residual of the differential/algebraic system
C             DELTA = G(T,Y,YPRIME)
C         DELTA is a vector of length NEQ which is output from RES.
C
C         Subroutine RES must not alter T, Y, YPRIME, or CJ.
C         You must declare the name RES in an EXTERNAL
C         statement in your program that calls DDASPK.
C         You must dimension Y, YPRIME, and DELTA in RES.
C
C         The input argument CJ can be ignored, or used to rescale
C         constraint equations in the system (see Ref. 2, p. 145).
C         Note: In this respect, DDASPK is not downward-compatible
C         with DDASSL, which does not have the RES argument CJ.
C
C         IRES is an integer flag which is always equal to zero
C         on input.  Subroutine RES should alter IRES only if it
C         encounters an illegal value of Y or a stop condition.
C         Set IRES = -1 if an input value is illegal, and DDASPK
C         will try to solve the problem without getting IRES = -1.
C         If IRES = -2, DDASPK will return control to the calling
C         program with IDID = -11.
C
C         RPAR and IPAR are real and integer parameter arrays which
C         you can use for communication between your calling program
C         and subroutine RES. They are not altered by DDASPK. If you
C         do not need RPAR or IPAR, ignore these parameters by treat-
C         ing them as dummy arguments. If you do choose to use them,
C         dimension them in your calling program and in RES as arrays
C         of appropriate length.
C
C  NEQ -- Set it to the number of equations in the system (NEQ .GE. 1).
C
C  T -- Set it to the initial point of the integration. (T must be
C       a variable.)
C
C  Y(*) -- Set this array to the initial values of the NEQ solution
C          components at the initial point.  You must dimension Y of
C          length at least NEQ in your calling program.
C
C  YPRIME(*) -- Set this array to the initial values of the NEQ first
C               derivatives of the solution components at the initial
C               point.  You must dimension YPRIME at least NEQ in your
C               calling program. 
C
C  TOUT - Set it to the first point at which a solution is desired.
C         You cannot take TOUT = T.  Integration either forward in T
C         (TOUT .GT. T) or backward in T (TOUT .LT. T) is permitted.
C
C         The code advances the solution from T to TOUT using step
C         sizes which are automatically selected so as to achieve the
C         desired accuracy.  If you wish, the code will return with the
C         solution and its derivative at intermediate steps (the
C         intermediate-output mode) so that you can monitor them,
C         but you still must provide TOUT in accord with the basic
C         aim of the code.
C
C         The first step taken by the code is a critical one because
C         it must reflect how fast the solution changes near the
C         initial point.  The code automatically selects an initial
C         step size which is practically always suitable for the
C         problem.  By using the fact that the code will not step past
C         TOUT in the first step, you could, if necessary, restrict the
C         length of the initial step.
C
C         For some problems it may not be permissible to integrate
C         past a point TSTOP, because a discontinuity occurs there
C         or the solution or its derivative is not defined beyond
C         TSTOP.  When you have declared a TSTOP point (see INFO(4)
C         and RWORK(1)), you have told the code not to integrate past
C         TSTOP.  In this case any tout beyond TSTOP is invalid input.
C
C  INFO(*) - Use the INFO array to give the code more details about
C            how you want your problem solved.  This array should be
C            dimensioned of length 20, though DDASPK uses only the 
C            first 15 entries.  You must respond to all of the following
C            items, which are arranged as questions.  The simplest use
C            of DDASPK corresponds to setting all entries of INFO to 0.
C
C       INFO(1) - This parameter enables the code to initialize itself.
C              You must set it to indicate the start of every new 
C              problem.
C
C          **** Is this the first call for this problem ...
C                yes - set INFO(1) = 0
C                 no - not applicable here.
C                      See below for continuation calls.  ****
C
C       INFO(2) - How much accuracy you want of your solution
C              is specified by the error tolerances RTOL and ATOL.
C              The simplest use is to take them both to be scalars.
C              To obtain more flexibility, they can both be arrays.
C              The code must be told your choice.
C
C          **** Are both error tolerances RTOL, ATOL scalars ...
C                yes - set INFO(2) = 0
C                      and input scalars for both RTOL and ATOL
C                 no - set INFO(2) = 1
C                      and input arrays for both RTOL and ATOL ****
C
C       INFO(3) - The code integrates from T in the direction of TOUT
C              by steps.  If you wish, it will return the computed
C              solution and derivative at the next intermediate step
C              (the intermediate-output mode) or TOUT, whichever comes
C              first.  This is a good way to proceed if you want to
C              see the behavior of the solution.  If you must have
C              solutions at a great many specific TOUT points, this
C              code will compute them efficiently.
C
C          **** Do you want the solution only at
C               TOUT (and not at the next intermediate step) ...
C                yes - set INFO(3) = 0
C                 no - set INFO(3) = 1 ****
C
C       INFO(4) - To handle solutions at a great many specific
C              values TOUT efficiently, this code may integrate past
C              TOUT and interpolate to obtain the result at TOUT.
C              Sometimes it is not possible to integrate beyond some
C              point TSTOP because the equation changes there or it is
C              not defined past TSTOP.  Then you must tell the code
C              this stop condition.
C
C           **** Can the integration be carried out without any
C                restrictions on the independent variable T ...
C                 yes - set INFO(4) = 0
C                  no - set INFO(4) = 1
C                       and define the stopping point TSTOP by
C                       setting RWORK(1) = TSTOP ****
C
C       INFO(5) - used only when INFO(12) = 0 (direct methods).
C              To solve differential/algebraic systems you may wish
C              to use a matrix of partial derivatives of the
C              system of differential equations.  If you do not
C              provide a subroutine to evaluate it analytically (see
C              description of the item JAC in the call list), it will
C              be approximated by numerical differencing in this code.
C              Although it is less trouble for you to have the code
C              compute partial derivatives by numerical differencing,
C              the solution will be more reliable if you provide the
C              derivatives via JAC.  Usually numerical differencing is
C              more costly than evaluating derivatives in JAC, but
C              sometimes it is not - this depends on your problem.
C
C           **** Do you want the code to evaluate the partial deriv-
C                atives automatically by numerical differences ...
C                 yes - set INFO(5) = 0
C                  no - set INFO(5) = 1
C                       and provide subroutine JAC for evaluating the
C                       matrix of partial derivatives ****
C
C       INFO(6) - used only when INFO(12) = 0 (direct methods).
C              DDASPK will perform much better if the matrix of
C              partial derivatives, dG/dY + CJ*dG/dYPRIME (here CJ is
C              a scalar determined by DDASPK), is banded and the code
C              is told this.  In this case, the storage needed will be
C              greatly reduced, numerical differencing will be performed
C              much cheaper, and a number of important algorithms will
C              execute much faster.  The differential equation is said 
C              to have half-bandwidths ML (lower) and MU (upper) if 
C              equation i involves only unknowns Y(j) with
C                             i-ML .le. j .le. i+MU .
C              For all i=1,2,...,NEQ.  Thus, ML and MU are the widths
C              of the lower and upper parts of the band, respectively,
C              with the main diagonal being excluded.  If you do not
C              indicate that the equation has a banded matrix of partial
C              derivatives the code works with a full matrix of NEQ**2
C              elements (stored in the conventional way).  Computations
C              with banded matrices cost less time and storage than with
C              full matrices if  2*ML+MU .lt. NEQ.  If you tell the
C              code that the matrix of partial derivatives has a banded
C              structure and you want to provide subroutine JAC to
C              compute the partial derivatives, then you must be careful
C              to store the elements of the matrix in the special form
C              indicated in the description of JAC.
C
C          **** Do you want to solve the problem using a full (dense)
C               matrix (and not a special banded structure) ...
C                yes - set INFO(6) = 0
C                 no - set INFO(6) = 1
C                       and provide the lower (ML) and upper (MU)
C                       bandwidths by setting
C                       IWORK(1)=ML
C                       IWORK(2)=MU ****
C
C       INFO(7) - You can specify a maximum (absolute value of)
C              stepsize, so that the code will avoid passing over very
C              large regions.
C
C          ****  Do you want the code to decide on its own the maximum
C                stepsize ...
C                 yes - set INFO(7) = 0
C                  no - set INFO(7) = 1
C                       and define HMAX by setting
C                       RWORK(2) = HMAX ****
C
C       INFO(8) -  Differential/algebraic problems may occasionally
C              suffer from severe scaling difficulties on the first
C              step.  If you know a great deal about the scaling of 
C              your problem, you can help to alleviate this problem 
C              by specifying an initial stepsize H0.
C
C          ****  Do you want the code to define its own initial
C                stepsize ...
C                 yes - set INFO(8) = 0
C                  no - set INFO(8) = 1
C                       and define H0 by setting
C                       RWORK(3) = H0 ****
C
C       INFO(9) -  If storage is a severe problem, you can save some
C              storage by restricting the maximum method order MAXORD.
C              The default value is 5.  For each order decrease below 5,
C              the code requires NEQ fewer locations, but it is likely 
C              to be slower.  In any case, you must have 
C              1 .le. MAXORD .le. 5.
C          ****  Do you want the maximum order to default to 5 ...
C                 yes - set INFO(9) = 0
C                  no - set INFO(9) = 1
C                       and define MAXORD by setting
C                       IWORK(3) = MAXORD ****
C
C       INFO(10) - If you know that certain components of the
C              solutions to your equations are always nonnegative
C              (or nonpositive), it may help to set this
C              parameter.  There are three options that are
C              available:
C              1.  To have constraint checking only in the initial
C                  condition calculation.
C              2.  To enforce nonnegativity in Y during the integration.
C              3.  To enforce both options 1 and 2.
C
C              When selecting option 2 or 3, it is probably best to try the
C              code without using this option first, and only use
C              this option if that does not work very well.
C
C          ****  Do you want the code to solve the problem without
C                invoking any special inequality constraints ...
C                 yes - set INFO(10) = 0
C                  no - set INFO(10) = 1 to have option 1 enforced 
C                  no - set INFO(10) = 2 to have option 2 enforced
C                  no - set INFO(10) = 3 to have option 3 enforced ****
C
C                  If you have specified INFO(10) = 1 or 3, then you
C                  will also need to identify how each component of Y
C                  in the initial condition calculation is constrained.
C                  You must set:
C                  IWORK(50+I) = +1 if Y(I) must be .GE. 0,
C                  IWORK(50+I) = +2 if Y(I) must be .GT. 0,
C                  IWORK(50+I) = -1 if Y(I) must be .LE. 0, while
C                  IWORK(50+I) = -2 if Y(I) must be .LT. 0, while
C                  IWORK(50+I) =  0 if Y(I) is not constrained.
C
C       INFO(11) - DDASPK normally requires the initial T, Y, and
C              YPRIME to be consistent.  That is, you must have
C              G(T,Y,YPRIME) = 0 at the initial T.  If you do not know
C              the initial conditions precisely, in some cases
C              DDASPK may be able to compute it.
C
C              Denoting the differential variables in Y by Y_d
C              and the algebraic variables by Y_a, DDASPK can solve
C              one of two initialization problems:
C              1.  Given Y_d, calculate Y_a and Y'_d, or
C              2.  Given Y', calculate Y.
C              In either case, initial values for the given
C              components are input, and initial guesses for
C              the unknown components must also be provided as input.
C
C          ****  Are the initial T, Y, YPRIME consistent ...
C
C                 yes - set INFO(11) = 0
C                  no - set INFO(11) = 1 to calculate option 1 above,
C                    or set INFO(11) = 2 to calculate option 2 ****
C
C                  If you have specified INFO(11) = 1, then you
C                  will also need to identify  which are the
C                  differential and which are the algebraic
C                  components (algebraic components are components
C                  whose derivatives do not appear explicitly
C                  in the function G(T,Y,YPRIME)).  You must set:
C                  IWORK(LID+I) = +1 if Y(I) is a differential variable
C                  IWORK(LID+I) = -1 if Y(I) is an algebraic variable,
C                  where LID = 50 if INFO(10) = 0 or 2 and LID = 50+NEQ
C                  if INFO(10) = 1 or 3.
C
C       INFO(12) - Except for the addition of the RES argument CJ,
C              DDASPK by default is downward-compatible with DDASSL,
C              which uses only direct (dense or band) methods to solve 
C              the linear systems involved.  You must set INFO(12) to
C              indicate whether you want the direct methods or the
C              Krylov iterative method.
C          ****   Do you want DDASPK to use standard direct methods
C                 (dense or band) or the Krylov (iterative) method ...
C                   direct methods - set INFO(12) = 0.
C                   Krylov method  - set INFO(12) = 1,
C                       and check the settings of INFO(13) and INFO(15).
C
C       INFO(13) - used when INFO(12) = 1 (Krylov methods).  
C              DDASPK uses scalars MAXL, KMP, NRMAX, and EPLI for the
C              iterative solution of linear systems.  INFO(13) allows 
C              you to override the default values of these parameters.  
C              These parameters and their defaults are as follows:
C              MAXL = maximum number of iterations in the SPIGMR 
C                 algorithm (MAXL .le. NEQ).  The default is 
C                 MAXL = MIN(5,NEQ).
C              KMP = number of vectors on which orthogonalization is 
C                 done in the SPIGMR algorithm.  The default is 
C                 KMP = MAXL, which corresponds to complete GMRES 
C                 iteration, as opposed to the incomplete form.  
C              NRMAX = maximum number of restarts of the SPIGMR 
C                 algorithm per nonlinear iteration.  The default is
C                 NRMAX = 5.
C              EPLI = convergence test constant in SPIGMR algorithm.
C                 The default is EPLI = 0.05.
C              Note that the length of RWORK depends on both MAXL 
C              and KMP.  See the definition of LRW below.
C          ****   Are MAXL, KMP, and EPLI to be given their
C                 default values ...
C                  yes - set INFO(13) = 0
C                   no - set INFO(13) = 1,
C                        and set all of the following:
C                        IWORK(24) = MAXL (1 .le. MAXL .le. NEQ)
C                        IWORK(25) = KMP  (1 .le. KMP .le. MAXL)
C                        IWORK(26) = NRMAX  (NRMAX .ge. 0)
C                        RWORK(10) = EPLI (0 .lt. EPLI .lt. 1.0) ****
C
C        INFO(14) - used with INFO(11) > 0 (initial condition 
C               calculation is requested).  In this case, you may
C               request control to be returned to the calling program
C               immediately after the initial condition calculation,
C               before proceeding to the integration of the system
C               (e.g. to examine the computed Y and YPRIME).
C               If this is done, and if the initialization succeeded
C               (IDID = 4), you should reset INFO(11) to 0 for the
C               next call, to prevent the solver from repeating the 
C               initialization (and to avoid an infinite loop). 
C          ****   Do you want to proceed to the integration after
C                 the initial condition calculation is done ...
C                 yes - set INFO(14) = 0
C                  no - set INFO(14) = 1                        ****
C
C        INFO(15) - used when INFO(12) = 1 (Krylov methods).
C               When using preconditioning in the Krylov method,
C               you must supply a subroutine, PSOL, which solves the
C               associated linear systems using P.
C               The usage of DDASPK is simpler if PSOL can carry out
C               the solution without any prior calculation of data.
C               However, if some partial derivative data is to be
C               calculated in advance and used repeatedly in PSOL,
C               then you must supply a JAC routine to do this,
C               and set INFO(15) to indicate that JAC is to be called
C               for this purpose.  For example, P might be an
C               approximation to a part of the matrix A which can be
C               calculated and LU-factored for repeated solutions of
C               the preconditioner system.  The arrays WP and IWP
C               (described under JAC and PSOL) can be used to
C               communicate data between JAC and PSOL.
C          ****   Does PSOL operate with no prior preparation ...
C                 yes - set INFO(15) = 0 (no JAC routine)
C                  no - set INFO(15) = 1
C                       and supply a JAC routine to evaluate and
C                       preprocess any required Jacobian data.  ****
C
C         INFO(16) - option to exclude algebraic variables from
C               the error test.  
C          ****   Do you wish to control errors locally on
C                 all the variables...
C                 yes - set INFO(16) = 0
C                  no - set INFO(16) = 1
C HR 24.07.2008
C                set INFO(16) = 2 to scale components with stepsize H
C		                for stepsize control
C		 index 2 variables are scaled with H
C		index 3 variables are scaled wuth H^2
C	        The index 1,2,3 variables should appear in this order
C		The dimensions must be specified in IWORK(45), IWORK(46)
C		and IWORK(47)
C	        IWORK(45): dimension of index 1 variables (=NEQ for ODE)
C		IWORK(46): dimension of index 2 variables
C		IWORK(47): dimension of index 3 variables
C 
C                       If you have specified INFO(16) = 1, then you
C                       will also need to identify  which are the
C                       differential and which are the algebraic
C                       components (algebraic components are components
C                       whose derivatives do not appear explicitly
C                       in the function G(T,Y,YPRIME)).  You must set:
C                       IWORK(LID+I) = +1 if Y(I) is a differential 
C                                      variable, and
C                       IWORK(LID+I) = -1 if Y(I) is an algebraic
C                                      variable,
C                       where LID = 50 if INFO(10) = 0 or 2 and 
C                       LID = 50 + NEQ if INFO(10) = 1 or 3.
C
C       INFO(17) - used when INFO(11) > 0 (DDASPK is to do an 
C              initial condition calculation).
C              DDASPK uses several heuristic control quantities in the
C              initial condition calculation.  They have default values,
C              but can  also be set by the user using INFO(17).
C              These parameters and their defaults are as follows:
C              MXNIT  = maximum number of Newton iterations
C                 per Jacobian or preconditioner evaluation.
C                 The default is:
C                 MXNIT =  5 in the direct case (INFO(12) = 0), and
C                 MXNIT = 15 in the Krylov case (INFO(12) = 1).
C              MXNJ   = maximum number of Jacobian or preconditioner
C                 evaluations.  The default is:
C                 MXNJ = 6 in the direct case (INFO(12) = 0), and
C                 MXNJ = 2 in the Krylov case (INFO(12) = 1).
C              MXNH   = maximum number of values of the artificial
C                 stepsize parameter H to be tried if INFO(11) = 1.
C                 The default is MXNH = 5.
C                 NOTE: the maximum number of Newton iterations
C                 allowed in all is MXNIT*MXNJ*MXNH if INFO(11) = 1,
C                 and MXNIT*MXNJ if INFO(11) = 2.
C              LSOFF  = flag to turn off the linesearch algorithm
C                 (LSOFF = 0 means linesearch is on, LSOFF = 1 means
C                 it is turned off).  The default is LSOFF = 0.
C              STPTOL = minimum scaled step in linesearch algorithm.
C                 The default is STPTOL = (unit roundoff)**(2/3).
C              EPINIT = swing factor in the Newton iteration convergence
C                 test.  The test is applied to the residual vector,
C                 premultiplied by the approximate Jacobian (in the
C                 direct case) or the preconditioner (in the Krylov
C                 case).  For convergence, the weighted RMS norm of
C                 this vector (scaled by the error weights) must be
C                 less than EPINIT*EPCON, where EPCON = .33 is the
C                 analogous test constant used in the time steps.
C                 The default is EPINIT = .01.
C          ****   Are the initial condition heuristic controls to be 
C                 given their default values...
C                  yes - set INFO(17) = 0
C                   no - set INFO(17) = 1,
C                        and set all of the following:
C                        IWORK(32) = MXNIT (.GT. 0)
C                        IWORK(33) = MXNJ (.GT. 0)
C                        IWORK(34) = MXNH (.GT. 0)
C                        IWORK(35) = LSOFF ( = 0 or 1)
C                        RWORK(14) = STPTOL (.GT. 0.0)
C                        RWORK(15) = EPINIT (.GT. 0.0)  ****
C
C         INFO(18) - option to get extra printing in initial condition 
C                calculation.
C          ****   Do you wish to have extra printing...
C                 no  - set INFO(18) = 0
C                 yes - set INFO(18) = 1 for minimal printing, or
C                       set INFO(18) = 2 for full printing.
C                       If you have specified INFO(18) .ge. 1, data
C                       will be printed with the error handler routines.
C                       To print to a non-default unit number L, include
C                       the line  CALL XSETUN(L)  in your program.  ****
C
C   RTOL, ATOL -- You must assign relative (RTOL) and absolute (ATOL)
C               error tolerances to tell the code how accurately you
C               want the solution to be computed.  They must be defined
C               as variables because the code may change them.
C               you have two choices --
C                     Both RTOL and ATOL are scalars (INFO(2) = 0), or
C                     both RTOL and ATOL are vectors (INFO(2) = 1).
C               In either case all components must be non-negative.
C
C               The tolerances are used by the code in a local error
C               test at each step which requires roughly that
C                        abs(local error in Y(i)) .le. EWT(i) ,
C               where EWT(i) = RTOL*abs(Y(i)) + ATOL is an error weight 
C               quantity, for each vector component.
C               (More specifically, a root-mean-square norm is used to
C               measure the size of vectors, and the error test uses the
C               magnitude of the solution at the beginning of the step.)
C
C               The true (global) error is the difference between the
C               true solution of the initial value problem and the
C               computed approximation.  Practically all present day
C               codes, including this one, control the local error at
C               each step and do not even attempt to control the global
C               error directly.
C
C               Usually, but not always, the true accuracy of
C               the computed Y is comparable to the error tolerances.
C               This code will usually, but not always, deliver a more
C               accurate solution if you reduce the tolerances and
C               integrate again.  By comparing two such solutions you 
C               can get a fairly reliable idea of the true error in the
C               solution at the larger tolerances.
C
C               Setting ATOL = 0. results in a pure relative error test
C               on that component.  Setting RTOL = 0. results in a pure
C               absolute error test on that component.  A mixed test
C               with non-zero RTOL and ATOL corresponds roughly to a
C               relative error test when the solution component is
C               much bigger than ATOL and to an absolute error test
C               when the solution component is smaller than the
C               threshold ATOL.
C
C               The code will not attempt to compute a solution at an
C               accuracy unreasonable for the machine being used.  It
C               will advise you if you ask for too much accuracy and
C               inform you as to the maximum accuracy it believes
C               possible.
C
C  RWORK(*) -- a real work array, which should be dimensioned in your
C               calling program with a length equal to the value of
C               LRW (or greater).
C
C  LRW -- Set it to the declared length of the RWORK array.  The
C               minimum length depends on the options you have selected,
C               given by a base value plus additional storage as described
C               below.
C
C               If INFO(12) = 0 (standard direct method), the base value is
C               base = 50 + max(MAXORD+4,7)*NEQ.
C               The default value is MAXORD = 5 (see INFO(9)).  With the
C               default MAXORD, base = 50 + 9*NEQ.
C               Additional storage must be added to the base value for
C               any or all of the following options:
C                 if INFO(6) = 0 (dense matrix), add NEQ**2
C                 if INFO(6) = 1 (banded matrix), then
C                    if INFO(5) = 0, add (2*ML+MU+1)*NEQ + 2*(NEQ/(ML+MU+1)+1),
C                    if INFO(5) = 1, add (2*ML+MU+1)*NEQ,
C                 if INFO(16) = 1, add NEQ.
C
C              If INFO(12) = 1 (Krylov method), the base value is
C              base = 50 + (MAXORD+5)*NEQ + (MAXL+3+MIN0(1,MAXL-KMP))*NEQ +
C                      + (MAXL+3)*MAXL + 1 + LENWP.
C              See PSOL for description of LENWP.  The default values are:
C              MAXORD = 5 (see INFO(9)), MAXL = min(5,NEQ) and KMP = MAXL 
C              (see INFO(13)).
C              With the default values for MAXORD, MAXL and KMP,
C              base = 91 + 18*NEQ + LENWP.
C              Additional storage must be added to the base value for
C              any or all of the following options:
C                if INFO(16) = 1, add NEQ.
C
C
C  IWORK(*) -- an integer work array, which should be dimensioned in
C              your calling program with a length equal to the value
C              of LIW (or greater).
C
C  LIW -- Set it to the declared length of the IWORK array.  The
C             minimum length depends on the options you have selected,
C             given by a base value plus additional storage as described
C             below.
C
C             If INFO(12) = 0 (standard direct method), the base value is
C             base = 50 + NEQ.
C             IF INFO(10) = 1 or 3, add NEQ to the base value.
C             If INFO(11) = 1 or INFO(16) =1, add NEQ to the base value.
C
C             If INFO(12) = 1 (Krylov method), the base value is
C             base = 50 + LENIWP.
C             See PSOL for description of LENIWP.
C             IF INFO(10) = 1 or 3, add NEQ to the base value.
C             If INFO(11) = 1 or INFO(16) = 1, add NEQ to the base value.
C
C
C  RPAR, IPAR -- These are arrays of double precision and integer type,
C             respectively, which are available for you to use
C             for communication between your program that calls
C             DDASPK and the RES subroutine (and the JAC and PSOL
C             subroutines).  They are not altered by DDASPK.
C             If you do not need RPAR or IPAR, ignore these
C             parameters by treating them as dummy arguments.
C             If you do choose to use them, dimension them in
C             your calling program and in RES (and in JAC and PSOL)
C             as arrays of appropriate length.
C
C  JAC -- This is the name of a routine that you may supply
C         (optionally) that relates to the Jacobian matrix of the
C         nonlinear system that the code must solve at each T step.
C         The role of JAC (and its call sequence) depends on whether
C         a direct (INFO(12) = 0) or Krylov (INFO(12) = 1) method 
C         is selected.
C
C         **** INFO(12) = 0 (direct methods):
C           If you are letting the code generate partial derivatives
C           numerically (INFO(5) = 0), then JAC can be absent
C           (or perhaps a dummy routine to satisfy the loader).
C           Otherwise you must supply a JAC routine to compute
C           the matrix A = dG/dY + CJ*dG/dYPRIME.  It must have
C           the form
C
C           SUBROUTINE JAC (T, Y, YPRIME, PD, CJ, RPAR, IPAR)
C
C           The JAC routine must dimension Y, YPRIME, and PD (and RPAR
C           and IPAR if used).  CJ is a scalar which is input to JAC.
C           For the given values of T, Y, and YPRIME, the JAC routine
C           must evaluate the nonzero elements of the matrix A, and 
C           store these values in the array PD.  The elements of PD are 
C           set to zero before each call to JAC, so that only nonzero
C           elements need to be defined.
C           The way you store the elements into the PD array depends
C           on the structure of the matrix indicated by INFO(6).
C           *** INFO(6) = 0 (full or dense matrix) ***
C               Give PD a first dimension of NEQ.  When you evaluate the
C               nonzero partial derivatives of equation i (i.e. of G(i))
C               with respect to component j (of Y and YPRIME), you must
C               store the element in PD according to
C                  PD(i,j) = dG(i)/dY(j) + CJ*dG(i)/dYPRIME(j).
C           *** INFO(6) = 1 (banded matrix with half-bandwidths ML, MU
C                            as described under INFO(6)) ***
C               Give PD a first dimension of 2*ML+MU+1.  When you 
C               evaluate the nonzero partial derivatives of equation i 
C               (i.e. of G(i)) with respect to component j (of Y and 
C               YPRIME), you must store the element in PD according to 
C                  IROW = i - j + ML + MU + 1
C                  PD(IROW,j) = dG(i)/dY(j) + CJ*dG(i)/dYPRIME(j).
C
C          **** INFO(12) = 1 (Krylov method):
C            If you are not calculating Jacobian data in advance for use
C            in PSOL (INFO(15) = 0), JAC can be absent (or perhaps a
C            dummy routine to satisfy the loader).  Otherwise, you may
C            supply a JAC routine to compute and preprocess any parts of
C            of the Jacobian matrix  A = dG/dY + CJ*dG/dYPRIME that are
C            involved in the preconditioner matrix P.
C            It is to have the form
C
C            SUBROUTINE JAC (RES, IRES, NEQ, T, Y, YPRIME, REWT, SAVR,
C                            WK, H, CJ, WP, IWP, IER, RPAR, IPAR)
C
C           The JAC routine must dimension Y, YPRIME, REWT, SAVR, WK,
C           and (if used) WP, IWP, RPAR, and IPAR.
C           The Y, YPRIME, and SAVR arrays contain the current values
C           of Y, YPRIME, and the residual G, respectively.  
C           The array WK is work space of length NEQ.  
C           H is the step size.  CJ is a scalar, input to JAC, that is
C           normally proportional to 1/H.  REWT is an array of 
C           reciprocal error weights, 1/EWT(i), where EWT(i) is
C           RTOL*abs(Y(i)) + ATOL (unless you supplied routine DDAWTS
C           instead), for use in JAC if needed.  For example, if JAC
C           computes difference quotient approximations to partial
C           derivatives, the REWT array may be useful in setting the
C           increments used.  The JAC routine should do any
C           factorization operations called for, in preparation for
C           solving linear systems in PSOL.  The matrix P should
C           be an approximation to the Jacobian,
C           A = dG/dY + CJ*dG/dYPRIME.
C
C           WP and IWP are real and integer work arrays which you may
C           use for communication between your JAC routine and your
C           PSOL routine.  These may be used to store elements of the 
C           preconditioner P, or related matrix data (such as factored
C           forms).  They are not altered by DDASPK.
C           If you do not need WP or IWP, ignore these parameters by
C           treating them as dummy arguments.  If you do use them,
C           dimension them appropriately in your JAC and PSOL routines.
C           See the PSOL description for instructions on setting 
C           the lengths of WP and IWP.
C
C           On return, JAC should set the error flag IER as follows..
C             IER = 0    if JAC was successful,
C             IER .ne. 0 if JAC was unsuccessful (e.g. if Y or YPRIME
C                        was illegal, or a singular matrix is found).
C           (If IER .ne. 0, a smaller stepsize will be tried.)
C           IER = 0 on entry to JAC, so need be reset only on a failure.
C           If RES is used within JAC, then a nonzero value of IRES will
C           override any nonzero value of IER (see the RES description).
C
C         Regardless of the method type, subroutine JAC must not
C         alter T, Y(*), YPRIME(*), H, CJ, or REWT(*).
C         You must declare the name JAC in an EXTERNAL statement in
C         your program that calls DDASPK.
C
C PSOL --  This is the name of a routine you must supply if you have
C         selected a Krylov method (INFO(12) = 1) with preconditioning.
C         In the direct case (INFO(12) = 0), PSOL can be absent 
C         (a dummy routine may have to be supplied to satisfy the 
C         loader).  Otherwise, you must provide a PSOL routine to 
C         solve linear systems arising from preconditioning.
C         When supplied with INFO(12) = 1, the PSOL routine is to 
C         have the form
C
C         SUBROUTINE PSOL (NEQ, T, Y, YPRIME, SAVR, WK, CJ, WGHT,
C                          WP, IWP, B, EPLIN, IER, RPAR, IPAR)
C
C         The PSOL routine must solve linear systems of the form 
C         P*x = b where P is the left preconditioner matrix.
C
C         The right-hand side vector b is in the B array on input, and
C         PSOL must return the solution vector x in B.
C         The Y, YPRIME, and SAVR arrays contain the current values
C         of Y, YPRIME, and the residual G, respectively.  
C
C         Work space required by JAC and/or PSOL, and space for data to
C         be communicated from JAC to PSOL is made available in the form
C         of arrays WP and IWP, which are parts of the RWORK and IWORK
C         arrays, respectively.  The lengths of these real and integer
C         work spaces WP and IWP must be supplied in LENWP and LENIWP,
C         respectively, as follows..
C           IWORK(27) = LENWP = length of real work space WP
C           IWORK(28) = LENIWP = length of integer work space IWP.
C
C         WK is a work array of length NEQ for use by PSOL.
C         CJ is a scalar, input to PSOL, that is normally proportional
C         to 1/H (H = stepsize).  If the old value of CJ
C         (at the time of the last JAC call) is needed, it must have
C         been saved by JAC in WP.
C
C         WGHT is an array of weights, to be used if PSOL uses an
C         iterative method and performs a convergence test.  (In terms
C         of the argument REWT to JAC, WGHT is REWT/sqrt(NEQ).)
C         If PSOL uses an iterative method, it should use EPLIN
C         (a heuristic parameter) as the bound on the weighted norm of
C         the residual for the computed solution.  Specifically, the
C         residual vector R should satisfy
C              SQRT (SUM ( (R(i)*WGHT(i))**2 ) ) .le. EPLIN
C
C         PSOL must not alter NEQ, T, Y, YPRIME, SAVR, CJ, WGHT, EPLIN.
C
C         On return, PSOL should set the error flag IER as follows..
C           IER = 0 if PSOL was successful,
C           IER .lt. 0 if an unrecoverable error occurred, meaning
C                 control will be passed to the calling routine,
C           IER .gt. 0 if a recoverable error occurred, meaning that
C                 the step will be retried with the same step size
C                 but with a call to JAC to update necessary data,
C                 unless the Jacobian data is current, in which case
C                 the step will be retried with a smaller step size.
C           IER = 0 on entry to PSOL so need be reset only on a failure.
C
C         You must declare the name PSOL in an EXTERNAL statement in
C         your program that calls DDASPK.
C
C
C  OPTIONALLY REPLACEABLE SUBROUTINE:
C
C  DDASPK uses a weighted root-mean-square norm to measure the 
C  size of various error vectors.  The weights used in this norm
C  are set in the following subroutine:
C
C    SUBROUTINE DDAWTS (NEQ, IWT, RTOL, ATOL, Y, EWT, RPAR, IPAR)
C    DIMENSION RTOL(*), ATOL(*), Y(*), EWT(*), RPAR(*), IPAR(*)
C
C  A DDAWTS routine has been included with DDASPK which sets the
C  weights according to
C    EWT(I) = RTOL*ABS(Y(I)) + ATOL
C  in the case of scalar tolerances (IWT = 0) or
C    EWT(I) = RTOL(I)*ABS(Y(I)) + ATOL(I)
C  in the case of array tolerances (IWT = 1).  (IWT is INFO(2).)
C  In some special cases, it may be appropriate for you to define
C  your own error weights by writing a subroutine DDAWTS to be 
C  called instead of the version supplied.  However, this should 
C  be attempted only after careful thought and consideration. 
C  If you supply this routine, you may use the tolerances and Y 
C  as appropriate, but do not overwrite these variables.  You
C  may also use RPAR and IPAR to communicate data as appropriate.
C  ***Note: Aside from the values of the weights, the choice of 
C  norm used in DDASPK (weighted root-mean-square) is not subject
C  to replacement by the user.  In this respect, DDASPK is not
C  downward-compatible with the original DDASSL solver (in which
C  the norm routine was optionally user-replaceable).
C
C
C------OUTPUT - AFTER ANY RETURN FROM DDASPK----------------------------
C
C  The principal aim of the code is to return a computed solution at
C  T = TOUT, although it is also possible to obtain intermediate
C  results along the way.  To find out whether the code achieved its
C  goal or if the integration process was interrupted before the task
C  was completed, you must check the IDID parameter.
C
C
C   T -- The output value of T is the point to which the solution
C        was successfully advanced.
C
C   Y(*) -- contains the computed solution approximation at T.
C
C   YPRIME(*) -- contains the computed derivative approximation at T.
C
C   IDID -- reports what the code did, described as follows:
C
C                     *** TASK COMPLETED ***
C                Reported by positive values of IDID
C
C           IDID = 1 -- a step was successfully taken in the
C                   intermediate-output mode.  The code has not
C                   yet reached TOUT.
C
C           IDID = 2 -- the integration to TSTOP was successfully
C                   completed (T = TSTOP) by stepping exactly to TSTOP.
C
C           IDID = 3 -- the integration to TOUT was successfully
C                   completed (T = TOUT) by stepping past TOUT.
C                   Y(*) and YPRIME(*) are obtained by interpolation.
C
C           IDID = 4 -- the initial condition calculation, with
C                   INFO(11) > 0, was successful, and INFO(14) = 1.
C                   No integration steps were taken, and the solution
C                   is not considered to have been started.
C
C                    *** TASK INTERRUPTED ***
C                Reported by negative values of IDID
C
C           IDID = -1 -- a large amount of work has been expended
C                     (about 500 steps).
C
C           IDID = -2 -- the error tolerances are too stringent.
C
C           IDID = -3 -- the local error test cannot be satisfied
C                     because you specified a zero component in ATOL
C                     and the corresponding computed solution component
C                     is zero.  Thus, a pure relative error test is
C                     impossible for this component.
C
C           IDID = -5 -- there were repeated failures in the evaluation
C                     or processing of the preconditioner (in JAC).
C
C           IDID = -6 -- DDASPK had repeated error test failures on the
C                     last attempted step.
C
C           IDID = -7 -- the nonlinear system solver in the time integration
C                     could not converge.
C
C           IDID = -8 -- the matrix of partial derivatives appears
C                     to be singular (direct method).
C
C           IDID = -9 -- the nonlinear system solver in the time integration
C                     failed to achieve convergence, and there were repeated 
C                     error test failures in this step.
C
C           IDID =-10 -- the nonlinear system solver in the time integration 
C                     failed to achieve convergence because IRES was equal 
C                     to -1.
C
C           IDID =-11 -- IRES = -2 was encountered and control is
C                     being returned to the calling program.
C
C           IDID =-12 -- DDASPK failed to compute the initial Y, YPRIME.
C
C           IDID =-13 -- unrecoverable error encountered inside user's
C                     PSOL routine, and control is being returned to
C                     the calling program.
C
C           IDID =-14 -- the Krylov linear system solver could not 
C                     achieve convergence.
C
C           IDID =-15,..,-32 -- Not applicable for this code.
C
C                    *** TASK TERMINATED ***
C                reported by the value of IDID=-33
C
C           IDID = -33 -- the code has encountered trouble from which
C                   it cannot recover.  A message is printed
C                   explaining the trouble and control is returned
C                   to the calling program.  For example, this occurs
C                   when invalid input is detected.
C
C   RTOL, ATOL -- these quantities remain unchanged except when
C               IDID = -2.  In this case, the error tolerances have been
C               increased by the code to values which are estimated to
C               be appropriate for continuing the integration.  However,
C               the reported solution at T was obtained using the input
C               values of RTOL and ATOL.
C
C   RWORK, IWORK -- contain information which is usually of no interest
C               to the user but necessary for subsequent calls. 
C               However, you may be interested in the performance data
C               listed below.  These quantities are accessed in RWORK 
C               and IWORK but have internal mnemonic names, as follows..
C
C               RWORK(3)--contains H, the step size h to be attempted
C                        on the next step.
C
C               RWORK(4)--contains TN, the current value of the
C                        independent variable, i.e. the farthest point
C                        integration has reached.  This will differ 
C                        from T if interpolation has been performed 
C                        (IDID = 3).
C
C               RWORK(7)--contains HOLD, the stepsize used on the last
C                        successful step.  If INFO(11) = INFO(14) = 1,
C                        this contains the value of H used in the
C                        initial condition calculation.
C
C               IWORK(7)--contains K, the order of the method to be 
C                        attempted on the next step.
C
C               IWORK(8)--contains KOLD, the order of the method used
C                        on the last step.
C
C               IWORK(11)--contains NST, the number of steps (in T) 
C                        taken so far.
C
C               IWORK(12)--contains NRE, the number of calls to RES 
C                        so far.
C
C               IWORK(13)--contains NJE, the number of calls to JAC so
C                        far (Jacobian or preconditioner evaluations).
C
C               IWORK(14)--contains NETF, the total number of error test
C                        failures so far.
C
C               IWORK(15)--contains NCFN, the total number of nonlinear
C                        convergence failures so far (includes counts
C                        of singular iteration matrix or singular
C                        preconditioners).
C
C               IWORK(16)--contains NCFL, the number of convergence
C                        failures of the linear iteration so far.
C
C               IWORK(17)--contains LENIW, the length of IWORK actually
C                        required.  This is defined on normal returns 
C                        and on an illegal input return for
C                        insufficient storage.
C
C               IWORK(18)--contains LENRW, the length of RWORK actually
C                        required.  This is defined on normal returns 
C                        and on an illegal input return for
C                        insufficient storage.
C
C               IWORK(19)--contains NNI, the total number of nonlinear
C                        iterations so far (each of which calls a
C                        linear solver).
C
C               IWORK(20)--contains NLI, the total number of linear
C                        (Krylov) iterations so far.
C
C               IWORK(21)--contains NPS, the number of PSOL calls so
C                        far, for preconditioning solve operations or
C                        for solutions with the user-supplied method.
C
C               Note: The various counters in IWORK do not include 
C               counts during a call made with INFO(11) > 0 and
C               INFO(14) = 1.
C
C
C------INPUT - WHAT TO DO TO CONTINUE THE INTEGRATION  -----------------
C              (CALLS AFTER THE FIRST)
C
C     This code is organized so that subsequent calls to continue the
C     integration involve little (if any) additional effort on your
C     part.  You must monitor the IDID parameter in order to determine
C     what to do next.
C
C     Recalling that the principal task of the code is to integrate
C     from T to TOUT (the interval mode), usually all you will need
C     to do is specify a new TOUT upon reaching the current TOUT.
C
C     Do not alter any quantity not specifically permitted below.  In
C     particular do not alter NEQ, T, Y(*), YPRIME(*), RWORK(*), 
C     IWORK(*), or the differential equation in subroutine RES.  Any 
C     such alteration constitutes a new problem and must be treated 
C     as such, i.e. you must start afresh.
C
C     You cannot change from array to scalar error control or vice
C     versa (INFO(2)), but you can change the size of the entries of
C     RTOL or ATOL.  Increasing a tolerance makes the equation easier
C     to integrate.  Decreasing a tolerance will make the equation
C     harder to integrate and should generally be avoided.
C
C     You can switch from the intermediate-output mode to the
C     interval mode (INFO(3)) or vice versa at any time.
C
C     If it has been necessary to prevent the integration from going
C     past a point TSTOP (INFO(4), RWORK(1)), keep in mind that the
C     code will not integrate to any TOUT beyond the currently
C     specified TSTOP.  Once TSTOP has been reached, you must change
C     the value of TSTOP or set INFO(4) = 0.  You may change INFO(4)
C     or TSTOP at any time but you must supply the value of TSTOP in
C     RWORK(1) whenever you set INFO(4) = 1.
C
C     Do not change INFO(5), INFO(6), INFO(12-17) or their associated
C     IWORK/RWORK locations unless you are going to restart the code.
C
C                    *** FOLLOWING A COMPLETED TASK ***
C
C     If..
C     IDID = 1, call the code again to continue the integration
C                  another step in the direction of TOUT.
C
C     IDID = 2 or 3, define a new TOUT and call the code again.
C                  TOUT must be different from T.  You cannot change
C                  the direction of integration without restarting.
C
C     IDID = 4, reset INFO(11) = 0 and call the code again to begin
C                  the integration.  (If you leave INFO(11) > 0 and
C                  INFO(14) = 1, you may generate an infinite loop.)
C                  In this situation, the next call to DASPK is 
C                  considered to be the first call for the problem,
C                  in that all initializations are done.
C
C                    *** FOLLOWING AN INTERRUPTED TASK ***
C
C     To show the code that you realize the task was interrupted and
C     that you want to continue, you must take appropriate action and
C     set INFO(1) = 1.
C
C     If..
C     IDID = -1, the code has taken about 500 steps.  If you want to
C                  continue, set INFO(1) = 1 and call the code again.
C                  An additional 500 steps will be allowed.
C
C
C     IDID = -2, the error tolerances RTOL, ATOL have been increased
C                  to values the code estimates appropriate for
C                  continuing.  You may want to change them yourself.
C                  If you are sure you want to continue with relaxed
C                  error tolerances, set INFO(1) = 1 and call the code
C                  again.
C
C     IDID = -3, a solution component is zero and you set the
C                  corresponding component of ATOL to zero.  If you
C                  are sure you want to continue, you must first alter
C                  the error criterion to use positive values of ATOL 
C                  for those components corresponding to zero solution
C                  components, then set INFO(1) = 1 and call the code
C                  again.
C
C     IDID = -4  --- cannot occur with this code.
C
C     IDID = -5, your JAC routine failed with the Krylov method.  Check
C                  for errors in JAC and restart the integration.
C
C     IDID = -6, repeated error test failures occurred on the last
C                  attempted step in DDASPK.  A singularity in the
C                  solution may be present.  If you are absolutely
C                  certain you want to continue, you should restart
C                  the integration.  (Provide initial values of Y and
C                  YPRIME which are consistent.)
C
C     IDID = -7, repeated convergence test failures occurred on the last
C                  attempted step in DDASPK.  An inaccurate or ill-
C                  conditioned Jacobian or preconditioner may be the
C                  problem.  If you are absolutely certain you want
C                  to continue, you should restart the integration.
C
C
C     IDID = -8, the matrix of partial derivatives is singular, with
C                  the use of direct methods.  Some of your equations
C                  may be redundant.  DDASPK cannot solve the problem
C                  as stated.  It is possible that the redundant
C                  equations could be removed, and then DDASPK could
C                  solve the problem.  It is also possible that a
C                  solution to your problem either does not exist
C                  or is not unique.
C
C     IDID = -9, DDASPK had multiple convergence test failures, preceded
C                  by multiple error test failures, on the last
C                  attempted step.  It is possible that your problem is
C                  ill-posed and cannot be solved using this code.  Or,
C                  there may be a discontinuity or a singularity in the
C                  solution.  If you are absolutely certain you want to
C                  continue, you should restart the integration.
C
C     IDID = -10, DDASPK had multiple convergence test failures
C                  because IRES was equal to -1.  If you are
C                  absolutely certain you want to continue, you
C                  should restart the integration.
C
C     IDID = -11, there was an unrecoverable error (IRES = -2) from RES
C                  inside the nonlinear system solver.  Determine the
C                  cause before trying again.
C
C     IDID = -12, DDASPK failed to compute the initial Y and YPRIME
C                  vectors.  This could happen because the initial 
C                  approximation to Y or YPRIME was not very good, or
C                  because no consistent values of these vectors exist.
C                  The problem could also be caused by an inaccurate or
C                  singular iteration matrix, or a poor preconditioner.
C
C     IDID = -13, there was an unrecoverable error encountered inside 
C                  your PSOL routine.  Determine the cause before 
C                  trying again.
C
C     IDID = -14, the Krylov linear system solver failed to achieve
C                  convergence.  This may be due to ill-conditioning
C                  in the iteration matrix, or a singularity in the
C                  preconditioner (if one is being used).
C                  Another possibility is that there is a better
C                  choice of Krylov parameters (see INFO(13)).
C                  Possibly the failure is caused by redundant equations
C                  in the system, or by inconsistent equations.
C                  In that case, reformulate the system to make it
C                  consistent and non-redundant.
C
C     IDID = -15,..,-32 --- Cannot occur with this code.
C
C                       *** FOLLOWING A TERMINATED TASK ***
C
C     If IDID = -33, you cannot continue the solution of this problem.
C                  An attempt to do so will result in your run being
C                  terminated.
C
C  ---------------------------------------------------------------------
C
C***REFERENCES
C  1.  L. R. Petzold, A Description of DASSL: A Differential/Algebraic
C      System Solver, in Scientific Computing, R. S. Stepleman et al.
C      (Eds.), North-Holland, Amsterdam, 1983, pp. 65-68.
C  2.  K. E. Brenan, S. L. Campbell, and L. R. Petzold, Numerical 
C      Solution of Initial-Value Problems in Differential-Algebraic
C      Equations, Elsevier, New York, 1989.
C  3.  P. N. Brown and A. C. Hindmarsh, Reduced Storage Matrix Methods
C      in Stiff ODE Systems, J. Applied Mathematics and Computation,
C      31 (1989), pp. 40-91.
C  4.  P. N. Brown, A. C. Hindmarsh, and L. R. Petzold, Using Krylov
C      Methods in the Solution of Large-Scale Differential-Algebraic
C      Systems, SIAM J. Sci. Comp., 15 (1994), pp. 1467-1488.
C  5.  P. N. Brown, A. C. Hindmarsh, and L. R. Petzold, Consistent
C      Initial Condition Calculation for Differential-Algebraic
C      Systems, SIAM J. Sci. Comp. 19 (1998), pp. 1495-1512.
C
C***ROUTINES CALLED
C
C   The following are all the subordinate routines used by DDASPK.
C
C   DDASIC computes consistent initial conditions.
C   DYYPNW updates Y and YPRIME in linesearch for initial condition
C          calculation.
C   DDSTP  carries out one step of the integration.
C   DCNSTR/DCNST0 check the current solution for constraint violations.
C   DDAWTS sets error weight quantities.
C   DINVWT tests and inverts the error weights.
C   DDATRP performs interpolation to get an output solution.
C   DDWNRM computes the weighted root-mean-square norm of a vector.
C   D1MACH provides the unit roundoff of the computer.
C   XERRWD/XSETF/XSETUN/IXSAV is a package to handle error messages. 
C   DDASID nonlinear equation driver to initialize Y and YPRIME using
C          direct linear system solver methods.  Interfaces to Newton
C          solver (direct case).
C   DNSID  solves the nonlinear system for unknown initial values by
C          modified Newton iteration and direct linear system methods.
C   DLINSD carries out linesearch algorithm for initial condition
C          calculation (direct case).
C   DFNRMD calculates weighted norm of preconditioned residual in
C          initial condition calculation (direct case).
C   DNEDD  nonlinear equation driver for direct linear system solver
C          methods.  Interfaces to Newton solver (direct case).
C   DMATD  assembles the iteration matrix (direct case).
C   DNSD   solves the associated nonlinear system by modified
C          Newton iteration and direct linear system methods.
C   DSLVD  interfaces to linear system solver (direct case).
C   DDASIK nonlinear equation driver to initialize Y and YPRIME using
C          Krylov iterative linear system methods.  Interfaces to
C          Newton solver (Krylov case).
C   DNSIK  solves the nonlinear system for unknown initial values by
C          Newton iteration and Krylov iterative linear system methods.
C   DLINSK carries out linesearch algorithm for initial condition
C          calculation (Krylov case).
C   DFNRMK calculates weighted norm of preconditioned residual in
C          initial condition calculation (Krylov case).
C   DNEDK  nonlinear equation driver for iterative linear system solver
C          methods.  Interfaces to Newton solver (Krylov case).
C   DNSK   solves the associated nonlinear system by Inexact Newton
C          iteration and (linear) Krylov iteration.
C   DSLVK  interfaces to linear system solver (Krylov case).
C   DSPIGM solves a linear system by SPIGMR algorithm.
C   DATV1   computes matrix-vector product in Krylov algorithm.
C   DORTH  performs orthogonalization of Krylov basis vectors.
C   DHEQR  performs QR factorization of Hessenberg matrix.
C   DHELS  finds least-squares solution of Hessenberg linear system.
C   DGEFA, DGESL, DGBFA, DGBSL are LINPACK routines for solving 
C          linear systems (dense or band direct methods).
C   DAXPY, DCOPY, DDOT, DNRM2, DSCAL are Basic Linear Algebra (BLAS)
C          routines.
C
C The routines called directly by DDASPK are:
C   DCNST0, DDAWTS, DINVWT, D1MACH, DDWNRM, DDASIC, DDATRP, DDSTP,
C   XERRWD
C
C***END PROLOGUE DDASPK
C
C
      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
      LOGICAL DONE, LAVL, LCFN, LCFL, LWARN
      DIMENSION Y(*),YPRIME(*)
      DIMENSION INFO(20)
      DIMENSION RWORK(LRW),IWORK(LIW)
      DIMENSION RTOL(*),ATOL(*)
      DIMENSION RPAR(*),IPAR(*)
      CHARACTER MSG*80
      EXTERNAL  RES, JAC, PSOL, DDASID, DDASIK, DNEDD, DNEDK
C
C     Set pointers into IWORK.
C     HR 18/07/2008 extended basis of IWORK to 50 and added LIND1..3   
C
      PARAMETER (LML=1, LMU=2, LMTYPE=4, 
     *   LIWM=1, LMXORD=3, LJCALC=5, LPHASE=6, LK=7, LKOLD=8,
     *   LNS=9, LNSTL=10, LNST=11, LNRE=12, LNJE=13, LETF=14, LNCFN=15,
     *   LNCFL=16, LNIW=17, LNRW=18, LNNI=19, LNLI=20, LNPS=21,
     *   LNPD=22, LMITER=23, LMAXL=24, LKMP=25, LNRMAX=26, LLNWP=27,
     *   LLNIWP=28, LLOCWP=29, LLCIWP=30, LKPRIN=31,
     *   LMXNIT=32, LMXNJ=33, LMXNH=34, LLSOFF=35, LIND1=45, LIND2=46,
     *   LIND3=47, LICNS=51)
C
C     Set pointers into RWORK.
C
      PARAMETER (LTSTOP=1, LHMAX=2, LH=3, LTN=4, LCJ=5, LCJOLD=6,
     *   LHOLD=7, LS=8, LROUND=9, LEPLI=10, LSQRN=11, LRSQRN=12,
     *   LEPCON=13, LSTOL=14, LEPIN=15,
     *   LALPHA=21, LBETA=27, LGAMMA=33, LPSI=39, LSIGMA=45, LDELTA=51)
C
      SAVE LID, LENID, NONNEG, NCPHI
C
C
C***FIRST EXECUTABLE STATEMENT  DDASPK
C
C
      IF(INFO(1).NE.0) GO TO 100
C
C-----------------------------------------------------------------------
C     This block is executed for the initial call only.
C     It contains checking of inputs and initializations.
C-----------------------------------------------------------------------
C
C     First check INFO array to make sure all elements of INFO
C     Are within the proper range.  (INFO(1) is checked later, because
C     it must be tested on every call.) ITEMP holds the location
C     within INFO which may be out of range.
C
      DO 10 I=2,9
         ITEMP = I
         IF (INFO(I) .NE. 0 .AND. INFO(I) .NE. 1) GO TO 701
 10      CONTINUE
      ITEMP = 10
      IF(INFO(10).LT.0 .OR. INFO(10).GT.3) GO TO 701
      ITEMP = 11
      IF(INFO(11).LT.0 .OR. INFO(11).GT.2) GO TO 701
      DO 15 I=12,15
         ITEMP = I
         IF (INFO(I) .NE. 0 .AND. INFO(I) .NE. 1) GO TO 701
 15      CONTINUE
      ITEMP = 16
      IF (INFO(16).LT.0 .OR. INFO(16).GT.2) GO TO 701
      ITEMP = 17
      IF (INFO(17) .NE. 0 .AND. INFO(17) .NE. 1) GO TO 701
      ITEMP = 18
      IF (INFO(18).LT.0 .OR. INFO(18).GT.2) GO TO 701

C
C     Check NEQ to see if it is positive.
      IF (NEQ .LE. 0) GO TO 702
C
C     Check dimension of index 1,2,3 variables (HR 24.07.2008)
      SUMLIND = IWORK(LIND1)+IWORK(LIND2)+IWORK(LIND3)
      IF(SUMLIND .NE. NEQ) THEN
        WRITE(6,*)' SUM IWORK(45,46,47) .NE. NEQ'
        RETURN
      ENDIF
C  
C     Check and compute maximum order.
C
      MXORD=5
      IF (INFO(9) .NE. 0) THEN
         MXORD=IWORK(LMXORD)
         IF (MXORD .LT. 1 .OR. MXORD .GT. 5) GO TO 703
         ENDIF
      IWORK(LMXORD)=MXORD
C
C     Set and/or check inputs for constraint checking (INFO(10) .NE. 0).
C     Set values for ICNFLG, NONNEG, and pointer LID.
C
      ICNFLG = 0
      NONNEG = 0
      LID = LICNS
      IF (INFO(10) .EQ. 0) GO TO 20
      IF (INFO(10) .EQ. 1) THEN
         ICNFLG = 1
         NONNEG = 0
         LID = LICNS + NEQ
      ELSEIF (INFO(10) .EQ. 2) THEN
         ICNFLG = 0
         NONNEG = 1
      ELSE
         ICNFLG = 1
         NONNEG = 1
         LID = LICNS + NEQ
      ENDIF
C
 20   CONTINUE
C
C     Set and/or check inputs for Krylov solver (INFO(12) .NE. 0).
C     If indicated, set default values for MAXL, KMP, NRMAX, and EPLI.
C     Otherwise, verify inputs required for iterative solver.
C
      IF (INFO(12) .EQ. 0) GO TO 25
C
      IWORK(LMITER) = INFO(12)
      IF (INFO(13) .EQ. 0) THEN
         IWORK(LMAXL) = MIN(5,NEQ)
         IWORK(LKMP) = IWORK(LMAXL)
         IWORK(LNRMAX) = 5
         RWORK(LEPLI) = 0.05D0
      ELSE
         IF(IWORK(LMAXL) .LT. 1 .OR. IWORK(LMAXL) .GT. NEQ) GO TO 720
         IF(IWORK(LKMP) .LT. 1 .OR. IWORK(LKMP) .GT. IWORK(LMAXL))
     1      GO TO 721
         IF(IWORK(LNRMAX) .LT. 0) GO TO 722
         IF(RWORK(LEPLI).LE.0.0D0 .OR. RWORK(LEPLI).GE.1.0D0)GO TO 723
         ENDIF
C
 25   CONTINUE
C
C     Set and/or check controls for the initial condition calculation
C     (INFO(11) .GT. 0).  If indicated, set default values.
C     Otherwise, verify inputs required for iterative solver.
C
      IF (INFO(11) .EQ. 0) GO TO 30
      IF (INFO(17) .EQ. 0) THEN
        IWORK(LMXNIT) = 5
        IF (INFO(12) .GT. 0) IWORK(LMXNIT) = 15
        IWORK(LMXNJ) = 6
        IF (INFO(12) .GT. 0) IWORK(LMXNJ) = 2
        IWORK(LMXNH) = 5
        IWORK(LLSOFF) = 0
        RWORK(LEPIN) = 0.01D0
      ELSE
        IF (IWORK(LMXNIT) .LE. 0) GO TO 725
        IF (IWORK(LMXNJ) .LE. 0) GO TO 725
        IF (IWORK(LMXNH) .LE. 0) GO TO 725
        LSOFF = IWORK(LLSOFF)
        IF (LSOFF .LT. 0 .OR. LSOFF .GT. 1) GO TO 725
        IF (RWORK(LEPIN) .LE. 0.0D0) GO TO 725
        ENDIF
C
 30   CONTINUE
C
C     Below is the computation and checking of the work array lengths
C     LENIW and LENRW, using direct methods (INFO(12) = 0) or
C     the Krylov methods (INFO(12) = 1).
C
      LENIC = 0
      IF (INFO(10) .EQ. 1 .OR. INFO(10) .EQ. 3) LENIC = NEQ
      LENID = 0
C LINE BELOW: 24.07.2008 HR Changed from INFO(16).EQ. 1 to .NE. 0 
      IF (INFO(11) .EQ. 1 .OR. INFO(16) .NE. 0) LENID = NEQ
      IF (INFO(12) .EQ. 0) THEN
C
C        Compute MTYPE, etc.  Check ML and MU.
C
         NCPHI = MAX(MXORD + 1, 4)
         IF(INFO(6).EQ.0) THEN 
            LENPD = NEQ**2
            LENRW = 50 + (NCPHI+3)*NEQ + LENPD
            IF(INFO(5).EQ.0) THEN
               IWORK(LMTYPE)=2
            ELSE
               IWORK(LMTYPE)=1
            ENDIF
         ELSE
            IF(IWORK(LML).LT.0.OR.IWORK(LML).GE.NEQ)GO TO 717
            IF(IWORK(LMU).LT.0.OR.IWORK(LMU).GE.NEQ)GO TO 718
            LENPD=(2*IWORK(LML)+IWORK(LMU)+1)*NEQ
            IF(INFO(5).EQ.0) THEN
               IWORK(LMTYPE)=5
               MBAND=IWORK(LML)+IWORK(LMU)+1
               MSAVE=(NEQ/MBAND)+1
               LENRW = 50 + (NCPHI+3)*NEQ + LENPD + 2*MSAVE
            ELSE
               IWORK(LMTYPE)=4
               LENRW = 50 + (NCPHI+3)*NEQ + LENPD
            ENDIF
         ENDIF
C
C        Compute LENIW, LENWP, LENIWP.
C                               HR 24.07.08 changed 40 to 50
         LENIW = 50 + LENIC + LENID + NEQ
         LENWP = 0
         LENIWP = 0
C
      ELSE IF (INFO(12) .EQ. 1)  THEN
         NCPHI = MXORD + 1
         MAXL = IWORK(LMAXL)
         LENWP = IWORK(LLNWP)
         LENIWP = IWORK(LLNIWP)
         LENPD = (MAXL+3+MIN0(1,MAXL-IWORK(LKMP)))*NEQ
     1         + (MAXL+3)*MAXL + 1 + LENWP
         LENRW = 50 + (MXORD+5)*NEQ + LENPD
         LENIW = 50 + LENIC + LENID + LENIWP
C
      ENDIF
      IF(INFO(16) .NE. 0) LENRW = LENRW + NEQ
C
C     Check lengths of RWORK and IWORK.
C
      IWORK(LNIW)=LENIW
      IWORK(LNRW)=LENRW
      IWORK(LNPD)=LENPD
      IWORK(LLOCWP) = LENPD-LENWP+1
      IF(LRW.LT.LENRW)GO TO 704
      IF(LIW.LT.LENIW)GO TO 705
C
C     Check ICNSTR for legality.
C
      IF (LENIC .GT. 0) THEN
        DO 40 I = 1,NEQ
          ICI = IWORK(LICNS-1+I)
          IF (ICI .LT. -2 .OR. ICI .GT. 2) GO TO 726
 40       CONTINUE
        ENDIF
C
C     Check Y for consistency with constraints.
C
      IF (LENIC .GT. 0) THEN
        CALL DCNST0(NEQ,Y,IWORK(LICNS),IRET)
        IF (IRET .NE. 0) GO TO 727
        ENDIF
C
C     Check ID for legality and set INDEX = 0 or 1.
C
      INDEX = 1
      IF (LENID .GT. 0) THEN
        INDEX = 0
        DO 50 I = 1,NEQ
          IDI = IWORK(LID-1+I)
          IF (IDI .NE. 1 .AND. IDI .NE. -1) GO TO 724
          IF (IDI .EQ. -1) INDEX = 1
 50       CONTINUE
        ENDIF
C
C     Check to see that TOUT is different from T.
C
      IF(TOUT .EQ. T)GO TO 719
C
C     Check HMAX.
C
      IF(INFO(7) .NE. 0) THEN
         HMAX = RWORK(LHMAX)
         IF (HMAX .LE. 0.0D0) GO TO 710
         ENDIF
C
C     Initialize counters and other flags.
C
      IWORK(LNST)=0
      IWORK(LNRE)=0
      IWORK(LNJE)=0
      IWORK(LETF)=0
      IWORK(LNCFN)=0
      IWORK(LNNI)=0
      IWORK(LNLI)=0
      IWORK(LNPS)=0
      IWORK(LNCFL)=0
      IWORK(LKPRIN)=INFO(18)
      IDID=1
      GO TO 200
C
C-----------------------------------------------------------------------
C     This block is for continuation calls only.
C     Here we check INFO(1), and if the last step was interrupted,
C     we check whether appropriate action was taken.
C-----------------------------------------------------------------------
C
100   CONTINUE
      IF(INFO(1).EQ.1)GO TO 110
      ITEMP = 1
      IF(INFO(1).NE.-1)GO TO 701
C
C     If we are here, the last step was interrupted by an error
C     condition from DDSTP, and appropriate action was not taken.
C     This is a fatal error.
C
      MSG = 'DASPK--  THE LAST STEP TERMINATED WITH A NEGATIVE'
      CALL XERRWD(MSG,49,201,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  VALUE (=I1) OF IDID AND NO APPROPRIATE'
      CALL XERRWD(MSG,47,202,0,1,IDID,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  ACTION WAS TAKEN. RUN TERMINATED'
      CALL XERRWD(MSG,41,203,1,0,0,0,0,0.0D0,0.0D0)
      RETURN
110   CONTINUE
C
C-----------------------------------------------------------------------
C     This block is executed on all calls.
C
C     Counters are saved for later checks of performance.
C     Then the error tolerance parameters are checked, and the
C     work array pointers are set.
C-----------------------------------------------------------------------
C
200   CONTINUE
C
C     Save counters for use later.
C
      IWORK(LNSTL)=IWORK(LNST)
      NLI0 = IWORK(LNLI)
      NNI0 = IWORK(LNNI)
      NCFN0 = IWORK(LNCFN)
      NCFL0 = IWORK(LNCFL)
      NWARN = 0
C
C     Check RTOL and ATOL.
C
      NZFLG = 0
      RTOLI = RTOL(1)
      ATOLI = ATOL(1)
      DO 210 I=1,NEQ
         IF (INFO(2) .EQ. 1) RTOLI = RTOL(I)
         IF (INFO(2) .EQ. 1) ATOLI = ATOL(I)
         IF (RTOLI .GT. 0.0D0 .OR. ATOLI .GT. 0.0D0) NZFLG = 1
         IF (RTOLI .LT. 0.0D0) GO TO 706
         IF (ATOLI .LT. 0.0D0) GO TO 707
210      CONTINUE
      IF (NZFLG .EQ. 0) GO TO 708
C
C     Set pointers to RWORK and IWORK segments.
C     For direct methods, SAVR is not used.
C
      IWORK(LLCIWP) = LID + LENID
      LSAVR = LDELTA
      IF (INFO(12) .NE. 0) LSAVR = LDELTA + NEQ
      LE = LSAVR + NEQ
      LWT = LE + NEQ
      LVT = LWT
      IF (INFO(16) .NE. 0) LVT = LWT + NEQ
      LPHI = LVT + NEQ
      LWM = LPHI + NCPHI*NEQ
      IF (INFO(1) .EQ. 1) GO TO 400
C
C-----------------------------------------------------------------------
C     This block is executed on the initial call only.
C     Set the initial step size, the error weight vector, and PHI.
C     Compute unknown initial components of Y and YPRIME, if requested.
C-----------------------------------------------------------------------
C
300   CONTINUE
      TN=T
      IDID=1
C
C     Set error weight array WT and altered weight array VT.
C
      CALL DDAWTS(NEQ,INFO(2),RTOL,ATOL,Y,RWORK(LWT),RPAR,IPAR)
      CALL DINVWT(NEQ,RWORK(LWT),IER)
      IF (IER .NE. 0) GO TO 713
      IF (INFO(16) .NE. 0) THEN
        DO 305 I = 1, NEQ
 305      RWORK(LVT+I-1) = MAX(IWORK(LID+I-1),0)*RWORK(LWT+I-1)
        ENDIF
C
C     Compute unit roundoff and HMIN.
C
      UROUND = D1MACH(4)
      RWORK(LROUND) = UROUND
      HMIN = 4.0D0*UROUND*MAX(ABS(T),ABS(TOUT))
C
C     Set/check STPTOL control for initial condition calculation.
C     
      IF (INFO(11) .NE. 0) THEN
        IF( INFO(17) .EQ. 0) THEN
          RWORK(LSTOL) = UROUND**.6667D0
        ELSE
          IF (RWORK(LSTOL) .LE. 0.0D0) GO TO 725
          ENDIF
        ENDIF
C
C     Compute EPCON and square root of NEQ and its reciprocal, used
C     inside iterative solver.
C
      RWORK(LEPCON) = 0.33D0
      FLOATN = NEQ
      RWORK(LSQRN) = SQRT(FLOATN)
      RWORK(LRSQRN) = 1.D0/RWORK(LSQRN)
C
C     Check initial interval to see that it is long enough.
C
      TDIST = ABS(TOUT - T)
      IF(TDIST .LT. HMIN) GO TO 714
C
C     Check H0, if this was input.
C
      IF (INFO(8) .EQ. 0) GO TO 310
         H0 = RWORK(LH)
         IF ((TOUT - T)*H0 .LT. 0.0D0) GO TO 711
         IF (H0 .EQ. 0.0D0) GO TO 712
         GO TO 320
310    CONTINUE
C
C     Compute initial stepsize, to be used by either
C     DDSTP or DDASIC, depending on INFO(11).
C
      H0 = 0.001D0*TDIST
      YPNORM = DDWNRM(NEQ,YPRIME,RWORK(LVT),RPAR,IPAR)
      IF (YPNORM .GT. 0.5D0/H0) H0 = 0.5D0/YPNORM
      H0 = SIGN(H0,TOUT-T)
C
C     Adjust H0 if necessary to meet HMAX bound.
C
320   IF (INFO(7) .EQ. 0) GO TO 330
         RH = ABS(H0)/RWORK(LHMAX)
         IF (RH .GT. 1.0D0) H0 = H0/RH
C
C     Check against TSTOP, if applicable.
C
330   IF (INFO(4) .EQ. 0) GO TO 340
         TSTOP = RWORK(LTSTOP)
         IF ((TSTOP - T)*H0 .LT. 0.0D0) GO TO 715
         IF ((T + H0 - TSTOP)*H0 .GT. 0.0D0) H0 = TSTOP - T
         IF ((TSTOP - TOUT)*H0 .LT. 0.0D0) GO TO 709
C
340   IF (INFO(11) .EQ. 0) GO TO 370
C
C     Compute unknown components of initial Y and YPRIME, depending
C     on INFO(11) and INFO(12).  INFO(12) represents the nonlinear
C     solver type (direct/Krylov).  Pass the name of the specific 
C     nonlinear solver, depending on INFO(12).  The location of the work
C     arrays SAVR, YIC, YPIC, PWK also differ in the two cases.
C     For use in stopping tests, pass TSCALE = TDIST if INDEX = 0.
C
      NWT = 1
      EPCONI = RWORK(LEPIN)*RWORK(LEPCON)
      TSCALE = 0.0D0
      IF (INDEX .EQ. 0) TSCALE = TDIST
350   IF (INFO(12) .EQ. 0) THEN
         LYIC = LPHI + 2*NEQ
         LYPIC = LYIC + NEQ
         LPWK = LYPIC
         CALL DDASIC(TN,Y,YPRIME,NEQ,INFO(11),IWORK(LID),
     *     RES,JAC,PSOL,H0,TSCALE,RWORK(LWT),NWT,IDID,RPAR,IPAR,
     *     RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
     *     RWORK(LYIC),RWORK(LYPIC),RWORK(LPWK),RWORK(LWM),IWORK(LIWM),
     *     RWORK(LROUND),RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
     *     EPCONI,RWORK(LSTOL),INFO(15),ICNFLG,IWORK(LICNS),DDASID)
      ELSE IF (INFO(12) .EQ. 1) THEN
         LYIC = LWM
         LYPIC = LYIC + NEQ
         LPWK = LYPIC + NEQ
         CALL DDASIC(TN,Y,YPRIME,NEQ,INFO(11),IWORK(LID),
     *     RES,JAC,PSOL,H0,TSCALE,RWORK(LWT),NWT,IDID,RPAR,IPAR,
     *     RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
     *     RWORK(LYIC),RWORK(LYPIC),RWORK(LPWK),RWORK(LWM),IWORK(LIWM),
     *     RWORK(LROUND),RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
     *     EPCONI,RWORK(LSTOL),INFO(15),ICNFLG,IWORK(LICNS),DDASIK)
      ENDIF
C
      IF (IDID .LT. 0) GO TO 600
C
C     DDASIC was successful.  If this was the first call to DDASIC,
C     update the WT array (with the current Y) and call it again.
C
      IF (NWT .EQ. 2) GO TO 355
      NWT = 2
      CALL DDAWTS(NEQ,INFO(2),RTOL,ATOL,Y,RWORK(LWT),RPAR,IPAR)
      CALL DINVWT(NEQ,RWORK(LWT),IER)
      IF (IER .NE. 0) GO TO 713
      GO TO 350
C
C     If INFO(14) = 1, return now with IDID = 4.
C
355   IF (INFO(14) .EQ. 1) THEN
        IDID = 4
        H = H0
        IF (INFO(11) .EQ. 1) RWORK(LHOLD) = H0
        GO TO 590
      ENDIF
C
C     Update the WT and VT arrays one more time, with the new Y.
C
      CALL DDAWTS(NEQ,INFO(2),RTOL,ATOL,Y,RWORK(LWT),RPAR,IPAR)
      CALL DINVWT(NEQ,RWORK(LWT),IER)
      IF (IER .NE. 0) GO TO 713
      IF (INFO(16) .NE. 0) THEN
        DO 357 I = 1, NEQ
 357      RWORK(LVT+I-1) = MAX(IWORK(LID+I-1),0)*RWORK(LWT+I-1)
        ENDIF
C
C     Reset the initial stepsize to be used by DDSTP.
C     Use H0, if this was input.  Otherwise, recompute H0,
C     and adjust it if necessary to meet HMAX bound.
C
      IF (INFO(8) .NE. 0) THEN
         H0 = RWORK(LH)
         GO TO 360
         ENDIF
C
      H0 = 0.001D0*TDIST
      YPNORM = DDWNRM(NEQ,YPRIME,RWORK(LVT),RPAR,IPAR)
      IF (YPNORM .GT. 0.5D0/H0) H0 = 0.5D0/YPNORM
      H0 = SIGN(H0,TOUT-T)
C
360   IF (INFO(7) .NE. 0) THEN
         RH = ABS(H0)/RWORK(LHMAX)
         IF (RH .GT. 1.0D0) H0 = H0/RH
         ENDIF
C
C     Check against TSTOP, if applicable.
C
      IF (INFO(4) .NE. 0) THEN
         TSTOP = RWORK(LTSTOP)
         IF ((T + H0 - TSTOP)*H0 .GT. 0.0D0) H0 = TSTOP - T
         ENDIF
C
C     Load H and RWORK(LH) with H0.
C
370   H = H0
      RWORK(LH) = H
C
C     Load Y and H*YPRIME into PHI(*,1) and PHI(*,2).
C
      ITEMP = LPHI + NEQ
      DO 380 I = 1,NEQ
         RWORK(LPHI + I - 1) = Y(I)
380      RWORK(ITEMP + I - 1) = H*YPRIME(I)
C
C    compute VT(H) 
C ADDED LINES BELOW:  HR 24.07.2008
      IF (INFO(16) .EQ. 2) THEN
        CALL DCALCVT(NEQ,H,INFO(2),RTOL,ATOL,Y,RWORK(LVT),IWORK(LIND1),
     *        IWORK(LIND2), IWORK(LIND3), RPAR, IPAR)
        CALL DINVWT(NEQ,RWORK(LVT),IER)
      IF (IER .NE. 0) GO TO 713
      ENDIF

      GO TO 500
C
C-----------------------------------------------------------------------
C     This block is for continuation calls only.
C     Its purpose is to check stop conditions before taking a step.
C     Adjust H if necessary to meet HMAX bound.
C-----------------------------------------------------------------------
C
400   CONTINUE
      UROUND=RWORK(LROUND)
      DONE = .FALSE.
      TN=RWORK(LTN)
      H=RWORK(LH)
      IF(INFO(7) .EQ. 0) GO TO 410
         RH = ABS(H)/RWORK(LHMAX)
         IF(RH .GT. 1.0D0) H = H/RH
410   CONTINUE
      IF(T .EQ. TOUT) GO TO 719
      IF((T - TOUT)*H .GT. 0.0D0) GO TO 711
      IF(INFO(4) .EQ. 1) GO TO 430
      IF(INFO(3) .EQ. 1) GO TO 420
      IF((TN-TOUT)*H.LT.0.0D0)GO TO 490
      CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      T=TOUT
      IDID = 3
      DONE = .TRUE.
      GO TO 490
420   IF((TN-T)*H .LE. 0.0D0) GO TO 490
      IF((TN - TOUT)*H .GE. 0.0D0) GO TO 425
      CALL DDATRP(TN,TN,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      T = TN
      IDID = 1
      DONE = .TRUE.
      GO TO 490
425   CONTINUE
      CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      T = TOUT
      IDID = 3
      DONE = .TRUE.
      GO TO 490
430   IF(INFO(3) .EQ. 1) GO TO 440
      TSTOP=RWORK(LTSTOP)
      IF((TN-TSTOP)*H.GT.0.0D0) GO TO 715
      IF((TSTOP-TOUT)*H.LT.0.0D0)GO TO 709
      IF((TN-TOUT)*H.LT.0.0D0)GO TO 450
      CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,IWORK(LKOLD),
     *   RWORK(LPHI),RWORK(LPSI))
      T=TOUT
      IDID = 3
      DONE = .TRUE.
      GO TO 490
440   TSTOP = RWORK(LTSTOP)
      IF((TN-TSTOP)*H .GT. 0.0D0) GO TO 715
      IF((TSTOP-TOUT)*H .LT. 0.0D0) GO TO 709
      IF((TN-T)*H .LE. 0.0D0) GO TO 450
      IF((TN - TOUT)*H .GE. 0.0D0) GO TO 445
      CALL DDATRP(TN,TN,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      T = TN
      IDID = 1
      DONE = .TRUE.
      GO TO 490
445   CONTINUE
      CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      T = TOUT
      IDID = 3
      DONE = .TRUE.
      GO TO 490
450   CONTINUE
C
C     Check whether we are within roundoff of TSTOP.
C
      IF(ABS(TN-TSTOP).GT.100.0D0*UROUND*
     *   (ABS(TN)+ABS(H)))GO TO 460
      CALL DDATRP(TN,TSTOP,Y,YPRIME,NEQ,IWORK(LKOLD),
     *  RWORK(LPHI),RWORK(LPSI))
      IDID=2
      T=TSTOP
      DONE = .TRUE.
      GO TO 490
460   TNEXT=TN+H
      IF((TNEXT-TSTOP)*H.LE.0.0D0)GO TO 490
      H=TSTOP-TN
      RWORK(LH)=H
C
490   IF (DONE) GO TO 590
C
C-----------------------------------------------------------------------
C     The next block contains the call to the one-step integrator DDSTP.
C     This is a looping point for the integration steps.
C     Check for too many steps.
C     Check for poor Newton/Krylov performance.
C     Update WT.  Check for too much accuracy requested.
C     Compute minimum stepsize.
C-----------------------------------------------------------------------
C
500   CONTINUE
C
C     Check for too many steps.
C
      IF((IWORK(LNST)-IWORK(LNSTL)).LT.500) GO TO 505
           IDID=-1
           GO TO 527
C
C Check for poor Newton/Krylov performance.
C
505   IF (INFO(12) .EQ. 0) GO TO 510
      NSTD = IWORK(LNST) - IWORK(LNSTL)
      NNID = IWORK(LNNI) - NNI0
      IF (NSTD .LT. 10 .OR. NNID .EQ. 0) GO TO 510
      AVLIN = REAL(IWORK(LNLI) - NLI0)/REAL(NNID)
      RCFN = REAL(IWORK(LNCFN) - NCFN0)/REAL(NSTD)
      RCFL = REAL(IWORK(LNCFL) - NCFL0)/REAL(NNID)
      FMAXL = IWORK(LMAXL)
      LAVL = AVLIN .GT. FMAXL
      LCFN = RCFN .GT. 0.9D0
      LCFL = RCFL .GT. 0.9D0
      LWARN = LAVL .OR. LCFN .OR. LCFL
      IF (.NOT.LWARN) GO TO 510
      NWARN = NWARN + 1
      IF (NWARN .GT. 10) GO TO 510
      IF (LAVL) THEN
        MSG = 'DASPK-- Warning. Poor iterative algorithm performance   '
        CALL XERRWD (MSG, 56, 501, 0, 0, 0, 0, 0, 0.0D0, 0.0D0)
        MSG = '      at T = R1. Average no. of linear iterations = R2  '
        CALL XERRWD (MSG, 56, 501, 0, 0, 0, 0, 2, TN, AVLIN)
        ENDIF
      IF (LCFN) THEN
        MSG = 'DASPK-- Warning. Poor iterative algorithm performance   '
        CALL XERRWD (MSG, 56, 502, 0, 0, 0, 0, 0, 0.0D0, 0.0D0)
        MSG = '      at T = R1. Nonlinear convergence failure rate = R2'
        CALL XERRWD (MSG, 56, 502, 0, 0, 0, 0, 2, TN, RCFN)
        ENDIF
      IF (LCFL) THEN
        MSG = 'DASPK-- Warning. Poor iterative algorithm performance   '
        CALL XERRWD (MSG, 56, 503, 0, 0, 0, 0, 0, 0.0D0, 0.0D0)
        MSG = '      at T = R1. Linear convergence failure rate = R2   '
        CALL XERRWD (MSG, 56, 503, 0, 0, 0, 0, 2, TN, RCFL)
        ENDIF
C
C     Update WT and VT, if this is not the first call.
C
510   CALL DDAWTS(NEQ,INFO(2),RTOL,ATOL,RWORK(LPHI),RWORK(LWT),
     *            RPAR,IPAR)
      CALL DINVWT(NEQ,RWORK(LWT),IER)
      IF (IER .NE. 0) THEN
        IDID = -3
        GO TO 527
        ENDIF
      IF (INFO(16) .NE. 0) THEN
        DO 515 I = 1, NEQ
 515      RWORK(LVT+I-1) = MAX(IWORK(LID+I-1),0)*RWORK(LWT+I-1)
        ENDIF
C
C     Test for too much accuracy requested.
C
      R = DDWNRM(NEQ,RWORK(LPHI),RWORK(LWT),RPAR,IPAR)*100.0D0*UROUND
      IF (R .LE. 1.0D0) GO TO 525
C
C     Multiply RTOL and ATOL by R and return.
C
      IF(INFO(2).EQ.1)GO TO 523
           RTOL(1)=R*RTOL(1)
           ATOL(1)=R*ATOL(1)
           IDID=-2
           GO TO 527
523   DO 524 I=1,NEQ
           RTOL(I)=R*RTOL(I)
524        ATOL(I)=R*ATOL(I)
      IDID=-2
      GO TO 527
525   CONTINUE
C
C     Compute minimum stepsize.
C
      HMIN=4.0D0*UROUND*MAX(ABS(TN),ABS(TOUT))
C
C     Test H vs. HMAX
      IF (INFO(7) .NE. 0) THEN
         RH = ABS(H)/RWORK(LHMAX)
         IF (RH .GT. 1.0D0) H = H/RH
         ENDIF

C     Compute VT(H) 
C ADDED LINES BELOW:  HR 24.07.2008
      IF (INFO(16) .EQ. 2) THEN
        CALL DCALCVT(NEQ,H,INFO(2),RTOL,ATOL,Y,RWORK(LVT),IWORK(LIND1),
     *        IWORK(LIND2), IWORK(LIND3), RPAR, IPAR)
        CALL DINVWT(NEQ,RWORK(LVT),IER)
      IF (IER .NE. 0) GO TO 713
      ENDIF

C
C     Call the one-step integrator.
C     Note that INFO(12) represents the nonlinear solver type.
C     Pass the required nonlinear solver, depending upon INFO(12).
C
      IF (INFO(12) .EQ. 0) THEN
         CALL DDSTP(TN,Y,YPRIME,NEQ,
     *      RES,JAC,PSOL,H,RWORK(LWT),RWORK(LVT),INFO(1),IDID,RPAR,IPAR,
     *      RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
     *      RWORK(LWM),IWORK(LIWM),
     *      RWORK(LALPHA),RWORK(LBETA),RWORK(LGAMMA),
     *      RWORK(LPSI),RWORK(LSIGMA),
     *      RWORK(LCJ),RWORK(LCJOLD),RWORK(LHOLD),RWORK(LS),HMIN,
     *      RWORK(LROUND), RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
     *      RWORK(LEPCON), IWORK(LPHASE),IWORK(LJCALC),INFO(15),
     *      IWORK(LK), IWORK(LKOLD),IWORK(LNS),NONNEG,INFO(12),
     *      DNEDD,
     *      INFO(2), INFO(16), RTOL, ATOL, IWORK(LIND1), 
     *      IWORK(LIND2), IWORK(LIND3))
      ELSE IF (INFO(12) .EQ. 1) THEN
         CALL DDSTP(TN,Y,YPRIME,NEQ,
     *      RES,JAC,PSOL,H,RWORK(LWT),RWORK(LVT),INFO(1),IDID,RPAR,IPAR,
     *      RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
     *      RWORK(LWM),IWORK(LIWM),
     *      RWORK(LALPHA),RWORK(LBETA),RWORK(LGAMMA),
     *      RWORK(LPSI),RWORK(LSIGMA),
     *      RWORK(LCJ),RWORK(LCJOLD),RWORK(LHOLD),RWORK(LS),HMIN,
     *      RWORK(LROUND), RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
     *      RWORK(LEPCON), IWORK(LPHASE),IWORK(LJCALC),INFO(15),
     *      IWORK(LK), IWORK(LKOLD),IWORK(LNS),NONNEG,INFO(12),
     *      DNEDK,
     *      INFO(2), INFO(16), RTOL, ATOL, IWORK(LIND1), 
     *      IWORK(LIND2), IWORK(LIND3))
      ENDIF
C
527   IF(IDID.LT.0)GO TO 600
C
C-----------------------------------------------------------------------
C     This block handles the case of a successful return from DDSTP
C     (IDID=1).  Test for stop conditions.
C-----------------------------------------------------------------------
C
      IF(INFO(4).NE.0)GO TO 540
           IF(INFO(3).NE.0)GO TO 530
             IF((TN-TOUT)*H.LT.0.0D0)GO TO 500
             CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,
     *         IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
             IDID=3
             T=TOUT
             GO TO 580
530          IF((TN-TOUT)*H.GE.0.0D0)GO TO 535
             T=TN
             IDID=1
             GO TO 580
535          CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,
     *         IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
             IDID=3
             T=TOUT
             GO TO 580
540   IF(INFO(3).NE.0)GO TO 550
      IF((TN-TOUT)*H.LT.0.0D0)GO TO 542
         CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,
     *     IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
         T=TOUT
         IDID=3
         GO TO 580
542   IF(ABS(TN-TSTOP).LE.100.0D0*UROUND*
     *   (ABS(TN)+ABS(H)))GO TO 545
      TNEXT=TN+H
      IF((TNEXT-TSTOP)*H.LE.0.0D0)GO TO 500
      H=TSTOP-TN
      GO TO 500
545   CALL DDATRP(TN,TSTOP,Y,YPRIME,NEQ,
     *  IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
      IDID=2
      T=TSTOP
      GO TO 580
550   IF((TN-TOUT)*H.GE.0.0D0)GO TO 555
      IF(ABS(TN-TSTOP).LE.100.0D0*UROUND*(ABS(TN)+ABS(H)))GO TO 552
      T=TN
      IDID=1
      GO TO 580
552   CALL DDATRP(TN,TSTOP,Y,YPRIME,NEQ,
     *  IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
      IDID=2
      T=TSTOP
      GO TO 580
555   CALL DDATRP(TN,TOUT,Y,YPRIME,NEQ,
     *   IWORK(LKOLD),RWORK(LPHI),RWORK(LPSI))
      T=TOUT
      IDID=3
580   CONTINUE
C
C-----------------------------------------------------------------------
C     All successful returns from DDASPK are made from this block.
C-----------------------------------------------------------------------
C
590   CONTINUE
      RWORK(LTN)=TN
      RWORK(LH)=H
      RETURN
C
C-----------------------------------------------------------------------
C     This block handles all unsuccessful returns other than for
C     illegal input.
C-----------------------------------------------------------------------
C
600   CONTINUE
      ITEMP = -IDID
      GO TO (610,620,630,700,655,640,650,660,670,675,
     *  680,685,690,695), ITEMP
C
C     The maximum number of steps was taken before
C     reaching tout.
C
610   MSG = 'DASPK--  AT CURRENT T (=R1)  500 STEPS'
      CALL XERRWD(MSG,38,610,0,0,0,0,1,TN,0.0D0)
      MSG = 'DASPK--  TAKEN ON THIS CALL BEFORE REACHING TOUT'
      CALL XERRWD(MSG,48,611,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Too much accuracy for machine precision.
C
620   MSG = 'DASPK--  AT T (=R1) TOO MUCH ACCURACY REQUESTED'
      CALL XERRWD(MSG,47,620,0,0,0,0,1,TN,0.0D0)
      MSG = 'DASPK--  FOR PRECISION OF MACHINE. RTOL AND ATOL'
      CALL XERRWD(MSG,48,621,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  WERE INCREASED TO APPROPRIATE VALUES'
      CALL XERRWD(MSG,45,622,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     WT(I) .LE. 0.0D0 for some I (not at start of problem).
C
630   MSG = 'DASPK--  AT T (=R1) SOME ELEMENT OF WT'
      CALL XERRWD(MSG,38,630,0,0,0,0,1,TN,0.0D0)
      MSG = 'DASPK--  HAS BECOME .LE. 0.0'
      CALL XERRWD(MSG,28,631,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Error test failed repeatedly or with H=HMIN.
C
640   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,640,0,0,0,0,2,TN,H)
      MSG='DASPK--  ERROR TEST FAILED REPEATEDLY OR WITH ABS(H)=HMIN'
      CALL XERRWD(MSG,57,641,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Nonlinear solver failed to converge repeatedly or with H=HMIN.
C
650   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,650,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  NONLINEAR SOLVER FAILED TO CONVERGE'
      CALL XERRWD(MSG,44,651,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  REPEATEDLY OR WITH ABS(H)=HMIN'
      CALL XERRWD(MSG,40,652,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     The preconditioner had repeated failures.
C
655   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,655,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  PRECONDITIONER HAD REPEATED FAILURES.'
      CALL XERRWD(MSG,46,656,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     The iteration matrix is singular.
C
660   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,660,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  ITERATION MATRIX IS SINGULAR.'
      CALL XERRWD(MSG,38,661,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Nonlinear system failure preceded by error test failures.
C
670   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,670,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  NONLINEAR SOLVER COULD NOT CONVERGE.'
      CALL XERRWD(MSG,45,671,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  ALSO, THE ERROR TEST FAILED REPEATEDLY.'
      CALL XERRWD(MSG,49,672,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Nonlinear system failure because IRES = -1.
C
675   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,675,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  NONLINEAR SYSTEM SOLVER COULD NOT CONVERGE'
      CALL XERRWD(MSG,51,676,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  BECAUSE IRES WAS EQUAL TO MINUS ONE'
      CALL XERRWD(MSG,44,677,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Failure because IRES = -2.
C
680   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2)'
      CALL XERRWD(MSG,40,680,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  IRES WAS EQUAL TO MINUS TWO'
      CALL XERRWD(MSG,36,681,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Failed to compute initial YPRIME.
C
685   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,685,0,0,0,0,0,0.0D0,0.0D0)
      MSG = 'DASPK--  INITIAL (Y,YPRIME) COULD NOT BE COMPUTED'
      CALL XERRWD(MSG,49,686,0,0,0,0,2,TN,H0)
      GO TO 700
C
C     Failure because IER was negative from PSOL.
C
690   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2)'
      CALL XERRWD(MSG,40,690,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  IER WAS NEGATIVE FROM PSOL'
      CALL XERRWD(MSG,35,691,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C     Failure because the linear system solver could not converge.
C
695   MSG = 'DASPK--  AT T (=R1) AND STEPSIZE H (=R2) THE'
      CALL XERRWD(MSG,44,695,0,0,0,0,2,TN,H)
      MSG = 'DASPK--  LINEAR SYSTEM SOLVER COULD NOT CONVERGE.'
      CALL XERRWD(MSG,50,696,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 700
C
C
700   CONTINUE
      INFO(1)=-1
      T=TN
      RWORK(LTN)=TN
      RWORK(LH)=H
      RETURN
C
C-----------------------------------------------------------------------
C     This block handles all error returns due to illegal input,
C     as detected before calling DDSTP.
C     First the error message routine is called.  If this happens
C     twice in succession, execution is terminated.
C-----------------------------------------------------------------------
C
701   MSG = 'DASPK--  ELEMENT (=I1) OF INFO VECTOR IS NOT VALID'
      CALL XERRWD(MSG,50,1,0,1,ITEMP,0,0,0.0D0,0.0D0)
      GO TO 750
702   MSG = 'DASPK--  NEQ (=I1) .LE. 0'
      CALL XERRWD(MSG,25,2,0,1,NEQ,0,0,0.0D0,0.0D0)
      GO TO 750
703   MSG = 'DASPK--  MAXORD (=I1) NOT IN RANGE'
      CALL XERRWD(MSG,34,3,0,1,MXORD,0,0,0.0D0,0.0D0)
      GO TO 750
704   MSG='DASPK--  RWORK LENGTH NEEDED, LENRW (=I1), EXCEEDS LRW (=I2)'
      CALL XERRWD(MSG,60,4,0,2,LENRW,LRW,0,0.0D0,0.0D0)
      GO TO 750
705   MSG='DASPK--  IWORK LENGTH NEEDED, LENIW (=I1), EXCEEDS LIW (=I2)'
      CALL XERRWD(MSG,60,5,0,2,LENIW,LIW,0,0.0D0,0.0D0)
      GO TO 750
706   MSG = 'DASPK--  SOME ELEMENT OF RTOL IS .LT. 0'
      CALL XERRWD(MSG,39,6,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
707   MSG = 'DASPK--  SOME ELEMENT OF ATOL IS .LT. 0'
      CALL XERRWD(MSG,39,7,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
708   MSG = 'DASPK--  ALL ELEMENTS OF RTOL AND ATOL ARE ZERO'
      CALL XERRWD(MSG,47,8,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
709   MSG='DASPK--  INFO(4) = 1 AND TSTOP (=R1) BEHIND TOUT (=R2)'
      CALL XERRWD(MSG,54,9,0,0,0,0,2,TSTOP,TOUT)
      GO TO 750
710   MSG = 'DASPK--  HMAX (=R1) .LT. 0.0'
      CALL XERRWD(MSG,28,10,0,0,0,0,1,HMAX,0.0D0)
      GO TO 750
711   MSG = 'DASPK--  TOUT (=R1) BEHIND T (=R2)'
      CALL XERRWD(MSG,34,11,0,0,0,0,2,TOUT,T)
      GO TO 750
712   MSG = 'DASPK--  INFO(8)=1 AND H0=0.0'
      CALL XERRWD(MSG,29,12,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
713   MSG = 'DASPK--  SOME ELEMENT OF WT IS .LE. 0.0'
      CALL XERRWD(MSG,39,13,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
714   MSG='DASPK-- TOUT (=R1) TOO CLOSE TO T (=R2) TO START INTEGRATION'
      CALL XERRWD(MSG,60,14,0,0,0,0,2,TOUT,T)
      GO TO 750
715   MSG = 'DASPK--  INFO(4)=1 AND TSTOP (=R1) BEHIND T (=R2)'
      CALL XERRWD(MSG,49,15,0,0,0,0,2,TSTOP,T)
      GO TO 750
717   MSG = 'DASPK--  ML (=I1) ILLEGAL. EITHER .LT. 0 OR .GT. NEQ'
      CALL XERRWD(MSG,52,17,0,1,IWORK(LML),0,0,0.0D0,0.0D0)
      GO TO 750
718   MSG = 'DASPK--  MU (=I1) ILLEGAL. EITHER .LT. 0 OR .GT. NEQ'
      CALL XERRWD(MSG,52,18,0,1,IWORK(LMU),0,0,0.0D0,0.0D0)
      GO TO 750
719   MSG = 'DASPK--  TOUT (=R1) IS EQUAL TO T (=R2)'
      CALL XERRWD(MSG,39,19,0,0,0,0,2,TOUT,T)
      GO TO 750
720   MSG = 'DASPK--  MAXL (=I1) ILLEGAL. EITHER .LT. 1 OR .GT. NEQ'
      CALL XERRWD(MSG,54,20,0,1,IWORK(LMAXL),0,0,0.0D0,0.0D0)
      GO TO 750
721   MSG = 'DASPK--  KMP (=I1) ILLEGAL. EITHER .LT. 1 OR .GT. MAXL'
      CALL XERRWD(MSG,54,21,0,1,IWORK(LKMP),0,0,0.0D0,0.0D0)
      GO TO 750
722   MSG = 'DASPK--  NRMAX (=I1) ILLEGAL. .LT. 0'
      CALL XERRWD(MSG,36,22,0,1,IWORK(LNRMAX),0,0,0.0D0,0.0D0)
      GO TO 750
723   MSG = 'DASPK--  EPLI (=R1) ILLEGAL. EITHER .LE. 0.D0 OR .GE. 1.D0'
      CALL XERRWD(MSG,58,23,0,0,0,0,1,RWORK(LEPLI),0.0D0)
      GO TO 750
724   MSG = 'DASPK--  ILLEGAL IWORK VALUE FOR INFO(11) .NE. 0'
      CALL XERRWD(MSG,48,24,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
725   MSG = 'DASPK--  ONE OF THE INPUTS FOR INFO(17) = 1 IS ILLEGAL'
      CALL XERRWD(MSG,54,25,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
726   MSG = 'DASPK--  ILLEGAL IWORK VALUE FOR INFO(10) .NE. 0'
      CALL XERRWD(MSG,48,26,0,0,0,0,0,0.0D0,0.0D0)
      GO TO 750
727   MSG = 'DASPK--  Y(I) AND IWORK(50+I) (I=I1) INCONSISTENT'
      CALL XERRWD(MSG,49,27,0,1,IRET,0,0,0.0D0,0.0D0)
      GO TO 750
750   IF(INFO(1).EQ.-1) GO TO 760
      INFO(1)=-1
      IDID=-33
      RETURN
760   MSG = 'DASPK--  REPEATED OCCURRENCES OF ILLEGAL INPUT'
      CALL XERRWD(MSG,46,701,0,0,0,0,0,0.0D0,0.0D0)
770   MSG = 'DASPK--  RUN TERMINATED. APPARENT INFINITE LOOP'
      CALL XERRWD(MSG,47,702,1,0,0,0,0,0.0D0,0.0D0)
      RETURN
C
C------END OF SUBROUTINE DDASPK-----------------------------------------
      END
C      SUBROUTINE DDASIC (X, Y, YPRIME, NEQ, ICOPT, ID, RES, JAC, PSOL,
C     *   H, TSCALE, WT, NIC, IDID, RPAR, IPAR, PHI, SAVR, DELTA, E,
C     *   YIC, YPIC, PWK, WM, IWM, UROUND, EPLI, SQRTN, RSQRTN,
C     *   EPCONI, STPTOL, JFLG, ICNFLG, ICNSTR, NLSIC)
CC
CC***BEGIN PROLOGUE  DDASIC
CC***REFER TO  DDASPK
CC***DATE WRITTEN   940628   (YYMMDD)
CC***REVISION DATE  941206   (YYMMDD)
CC***REVISION DATE  950714   (YYMMDD)
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DDASIC is a driver routine to compute consistent initial values
CC     for Y and YPRIME.  There are two different options:  
CC     Denoting the differential variables in Y by Y_d, and
CC     the algebraic variables by Y_a, the problem solved is either:
CC     1.  Given Y_d, calculate Y_a and Y_d', or
CC     2.  Given Y', calculate Y.
CC     In either case, initial values for the given components
CC     are input, and initial guesses for the unknown components
CC     must also be provided as input.
CC
CC     The external routine NLSIC solves the resulting nonlinear system.
CC
CC     The parameters represent
CC
CC     X  --        Independent variable.
CC     Y  --        Solution vector at X.
CC     YPRIME --    Derivative of solution vector.
CC     NEQ --       Number of equations to be integrated.
CC     ICOPT     -- Flag indicating initial condition option chosen.
CC                    ICOPT = 1 for option 1 above.
CC                    ICOPT = 2 for option 2.
CC     ID        -- Array of dimension NEQ, which must be initialized
CC                  if option 1 is chosen.
CC                    ID(i) = +1 if Y_i is a differential variable,
CC                    ID(i) = -1 if Y_i is an algebraic variable. 
CC     RES --       External user-supplied subroutine to evaluate the
CC                  residual.  See RES description in DDASPK prologue.
CC     JAC --       External user-supplied routine to update Jacobian
CC                  or preconditioner information in the nonlinear solver
CC                  (optional).  See JAC description in DDASPK prologue.
CC     PSOL --      External user-supplied routine to solve
CC                  a linear system using preconditioning. 
CC                  See PSOL in DDASPK prologue.
CC     H --         Scaling factor in iteration matrix.  DDASIC may 
CC                  reduce H to achieve convergence.
CC     TSCALE --    Scale factor in T, used for stopping tests if nonzero.
CC     WT --        Vector of weights for error criterion.
CC     NIC --       Input number of initial condition calculation call 
CC                  (= 1 or 2).
CC     IDID --      Completion code.  See IDID in DDASPK prologue.
CC     RPAR,IPAR -- Real and integer parameter arrays that
CC                  are used for communication between the
CC                  calling program and external user routines.
CC                  They are not altered by DNSK
CC     PHI --       Work space for DDASIC of length at least 2*NEQ.
CC     SAVR --      Work vector for DDASIC of length NEQ.
CC     DELTA --     Work vector for DDASIC of length NEQ.
CC     E --         Work vector for DDASIC of length NEQ.
CC     YIC,YPIC --  Work vectors for DDASIC, each of length NEQ.
CC     PWK --       Work vector for DDASIC of length NEQ.
CC     WM,IWM --    Real and integer arrays storing
CC                  information required by the linear solver.
CC     EPCONI --    Test constant for Newton iteration convergence.
CC     ICNFLG --    Flag showing whether constraints on Y are to apply.
CC     ICNSTR --    Integer array of length NEQ with constraint types.
CC
CC     The other parameters are for use internally by DDASIC.
CC
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   DCOPY, NLSIC
CC
CC***END PROLOGUE  DDASIC
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),ID(*),WT(*),PHI(NEQ,*)
C      DIMENSION SAVR(*),DELTA(*),E(*),YIC(*),YPIC(*),PWK(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*), ICNSTR(*)
C      EXTERNAL RES, JAC, PSOL, NLSIC
CC
C      PARAMETER (LCFN=15)
C      PARAMETER (LMXNH=34)
CC
CC The following parameters are data-loaded here:
CC     RHCUT  = factor by which H is reduced on retry of Newton solve.
CC     RATEMX = maximum convergence rate for which Newton iteration
CC              is considered converging.
CC
C      SAVE RHCUT, RATEMX
C      DATA RHCUT/0.1D0/, RATEMX/0.8D0/
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 1.
CC     Initializations.
CC     JSKIP is a flag set to 1 when NIC = 2 and NH = 1, to signal that
CC     the initial call to the JAC routine is to be skipped then.
CC     Save Y and YPRIME in PHI.  Initialize IDID, NH, and CJ.
CC-----------------------------------------------------------------------
CC
C      MXNH = IWM(LMXNH)
C      IDID = 1
C      NH = 1
C      JSKIP = 0
C      IF (NIC .EQ. 2) JSKIP = 1
C      CALL DCOPY (NEQ, Y, 1, PHI(1,1), 1)
C      CALL DCOPY (NEQ, YPRIME, 1, PHI(1,2), 1)
CC
C      IF (ICOPT .EQ. 2) THEN
C        CJ = 0.0D0 
C      ELSE
C        CJ = 1.0D0/H
C      ENDIF
CC
CC-----------------------------------------------------------------------
CC     BLOCK 2
CC     Call the nonlinear system solver to obtain
CC     consistent initial values for Y and YPRIME.
CC-----------------------------------------------------------------------
CC
C 200  CONTINUE
C      CALL NLSIC(X,Y,YPRIME,NEQ,ICOPT,ID,RES,JAC,PSOL,H,TSCALE,WT,
C     *   JSKIP,RPAR,IPAR,SAVR,DELTA,E,YIC,YPIC,PWK,WM,IWM,CJ,UROUND,
C     *   EPLI,SQRTN,RSQRTN,EPCONI,RATEMX,STPTOL,JFLG,ICNFLG,ICNSTR,
C     *   IERNLS)
CC
C      IF (IERNLS .EQ. 0) RETURN
CC
CC-----------------------------------------------------------------------
CC     BLOCK 3
CC     The nonlinear solver was unsuccessful.  Increment NCFN.
CC     Return with IDID = -12 if either
CC       IERNLS = -1: error is considered unrecoverable,
CC       ICOPT = 2: we are doing initialization problem type 2, or
CC       NH = MXNH: the maximum number of H values has been tried.
CC     Otherwise (problem 1 with IERNLS .GE. 1), reduce H and try again.
CC     If IERNLS > 1, restore Y and YPRIME to their original values.
CC-----------------------------------------------------------------------
CC
C      IWM(LCFN) = IWM(LCFN) + 1
C      JSKIP = 0
CC
C      IF (IERNLS .EQ. -1) GO TO 350
C      IF (ICOPT .EQ. 2) GO TO 350
C      IF (NH .EQ. MXNH) GO TO 350
CC
C      NH = NH + 1
C      H = H*RHCUT
C      CJ = 1.0D0/H
CC
C      IF (IERNLS .EQ. 1) GO TO 200
CC
C      CALL DCOPY (NEQ, PHI(1,1), 1, Y, 1)
C      CALL DCOPY (NEQ, PHI(1,2), 1, YPRIME, 1)
C      GO TO 200
CC
C 350  IDID = -12
C      RETURN
CC
CC------END OF SUBROUTINE DDASIC-----------------------------------------
C      END
C      SUBROUTINE DYYPNW (NEQ, Y, YPRIME, CJ, RL, P, ICOPT, ID, 
C     *                   YNEW, YPNEW)
CC
CC***BEGIN PROLOGUE  DYYPNW
CC***REFER TO  DLINSK
CC***DATE WRITTEN   940830   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DYYPNW calculates the new (Y,YPRIME) pair needed in the
CC     linesearch algorithm based on the current lambda value.  It is
CC     called by DLINSK and DLINSD.  Based on the ICOPT and ID values,
CC     the corresponding entry in Y or YPRIME is updated.
CC
CC     In addition to the parameters described in the calling programs,
CC     the parameters represent
CC
CC     P      -- Array of length NEQ that contains the current
CC               approximate Newton step.
CC     RL     -- Scalar containing the current lambda value.
CC     YNEW   -- Array of length NEQ containing the updated Y vector.
CC     YPNEW  -- Array of length NEQ containing the updated YPRIME
CC               vector.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED (NONE)
CC
CC***END PROLOGUE  DYYPNW
CC
CC
C      IMPLICIT DOUBLE PRECISION (A-H,O-Z)
C      DIMENSION Y(*), YPRIME(*), YNEW(*), YPNEW(*), ID(*), P(*)
CC
C      IF (ICOPT .EQ. 1) THEN
C         DO 10 I=1,NEQ
C            IF(ID(I) .LT. 0) THEN
C               YNEW(I) = Y(I) - RL*P(I)
C               YPNEW(I) = YPRIME(I)
C            ELSE
C               YNEW(I) = Y(I)
C               YPNEW(I) = YPRIME(I) - RL*CJ*P(I)
C            ENDIF
C 10      CONTINUE
C      ELSE
C         DO 20 I = 1,NEQ
C            YNEW(I) = Y(I) - RL*P(I)
C            YPNEW(I) = YPRIME(I)
C 20      CONTINUE
C      ENDIF
C      RETURN
CC----------------------- END OF SUBROUTINE DYYPNW ----------------------
C      END
C      SUBROUTINE DDSTP(X,Y,YPRIME,NEQ,RES,JAC,PSOL,H,WT,VT,
C     *  JSTART,IDID,RPAR,IPAR,PHI,SAVR,DELTA,E,WM,IWM,
C     *  ALPHA,BETA,GAMMA,PSI,SIGMA,CJ,CJOLD,HOLD,S,HMIN,UROUND,
C     *  EPLI,SQRTN,RSQRTN,EPCON,IPHASE,JCALC,JFLG,K,KOLD,NS,NONNEG,
C     *  NTYPE,NLS,INFO2,INFO16,RTOL,ATOL,NRIND1,NRIND2,NRIND3)
CC
CC***BEGIN PROLOGUE  DDSTP
CC***REFER TO  DDASPK
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940909   (YYMMDD) (Reset PSI(1), PHI(*,2) at 690)
CC
CC***REVISION DATE  080727   (YYMMDD) H dependened error test added (R. Huber)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DDSTP solves a system of differential/algebraic equations of 
CC     the form G(X,Y,YPRIME) = 0, for one step (normally from X to X+H).
CC
CC     The methods used are modified divided difference, fixed leading 
CC     coefficient forms of backward differentiation formulas.  
CC     The code adjusts the stepsize and order to control the local error
CC     per step.
CC
CC
CC     The parameters represent
CC     X  --        Independent variable.
CC     Y  --        Solution vector at X.
CC     YPRIME --    Derivative of solution vector
CC                  after successful step.
CC     NEQ --       Number of equations to be integrated.
CC     RES --       External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     JAC --       External user-supplied routine to update
CC                  Jacobian or preconditioner information in the
CC                  nonlinear solver.  See JAC description in DDASPK
CC                  prologue.
CC     PSOL --      External user-supplied routine to solve
CC                  a linear system using preconditioning. 
CC                  (This is optional).  See PSOL in DDASPK prologue.
CC     H --         Appropriate step size for next step.
CC                  Normally determined by the code.
CC     WT --        Vector of weights for error criterion used in Newton test.
CC     VT --        Masked vector of weights used in error test.
CC     JSTART --    Integer variable set 0 for
CC                  first step, 1 otherwise.
CC     IDID --      Completion code returned from the nonlinear solver.
CC                  See IDID description in DDASPK prologue.
CC     RPAR,IPAR -- Real and integer parameter arrays that
CC                  are used for communication between the
CC                  calling program and external user routines.
CC                  They are not altered by DNSK
CC     PHI --       Array of divided differences used by
CC                  DDSTP. The length is NEQ*(K+1), where
CC                  K is the maximum order.
CC     SAVR --      Work vector for DDSTP of length NEQ.
CC     DELTA,E --   Work vectors for DDSTP of length NEQ.
CC     WM,IWM --    Real and integer arrays storing
CC                  information required by the linear solver.
CC
CC     The other parameters are information
CC     which is needed internally by DDSTP to
CC     continue from step to step.
CC
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   NLS, DDWNRM, DDATRP
CC
CC***END PROLOGUE  DDSTP
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*),VT(*), RTOL(*), ATOL(*)
C      DIMENSION PHI(NEQ,*),SAVR(*),DELTA(*),E(*)
C      DIMENSION WM(*),IWM(*)
C      DIMENSION PSI(*),ALPHA(*),BETA(*),GAMMA(*),SIGMA(*)
C      DIMENSION RPAR(*),IPAR(*)
C      EXTERNAL  RES, JAC, PSOL, NLS
CC
C      PARAMETER (LMXORD=3)
C      PARAMETER (LNST=11, LETF=14, LCFN=15)
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 1.
CC     Initialize.  On the first call, set
CC     the order to 1 and initialize
CC     other variables.
CC-----------------------------------------------------------------------
CC
CC     Initializations for all calls
CC
C      XOLD=X
C      NCF=0
C      NEF=0
C      IF(JSTART .NE. 0) GO TO 120
CC
CC     If this is the first step, perform
CC     other initializations
CC
C      K=1
C      KOLD=0
C      HOLD=0.0D0
C      PSI(1)=H
C      CJ = 1.D0/H
C      IPHASE = 0
C      NS=0
C120   CONTINUE
CC
CC
CC
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 2
CC     Compute coefficients of formulas for
CC     this step.
CC-----------------------------------------------------------------------
C200   CONTINUE
C      KP1=K+1
C      KP2=K+2
C      KM1=K-1
C      IF(H.NE.HOLD.OR.K .NE. KOLD) NS = 0
C      NS=MIN0(NS+1,KOLD+2)
C      NSP1=NS+1
C      IF(KP1 .LT. NS)GO TO 230
CC
C      BETA(1)=1.0D0
C      ALPHA(1)=1.0D0
C      TEMP1=H
C      GAMMA(1)=0.0D0
C      SIGMA(1)=1.0D0
C      DO 210 I=2,KP1
C         TEMP2=PSI(I-1)
C         PSI(I-1)=TEMP1
C         BETA(I)=BETA(I-1)*PSI(I-1)/TEMP2
C         TEMP1=TEMP2+H
C         ALPHA(I)=H/TEMP1
C         SIGMA(I)=(I-1)*SIGMA(I-1)*ALPHA(I)
C         GAMMA(I)=GAMMA(I-1)+ALPHA(I-1)/H
C210      CONTINUE
C      PSI(KP1)=TEMP1
C230   CONTINUE
CC
CC     Compute ALPHAS, ALPHA0
CC
C      ALPHAS = 0.0D0
C      ALPHA0 = 0.0D0
C      DO 240 I = 1,K
C        ALPHAS = ALPHAS - 1.0D0/I
C        ALPHA0 = ALPHA0 - ALPHA(I)
C240     CONTINUE
CC
CC     Compute leading coefficient CJ
CC
C      CJLAST = CJ
C      CJ = -ALPHAS/H
CC
CC     Compute variable stepsize error coefficient CK
CC
C      CK = ABS(ALPHA(KP1) + ALPHAS - ALPHA0)
C      CK = MAX(CK,ALPHA(KP1))
CC
CC     Change PHI to PHI STAR
CC
C      IF(KP1 .LT. NSP1) GO TO 280
C      DO 270 J=NSP1,KP1
C         DO 260 I=1,NEQ
C260         PHI(I,J)=BETA(J)*PHI(I,J)
C270      CONTINUE
C280   CONTINUE
CC
CC     Update time
CC
C      X=X+H
CC
CC     Initialize IDID to 1
CC
C      IDID = 1
CC
CC
CC
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 3
CC     Call the nonlinear system solver to obtain the solution and
CC     derivative.
CC-----------------------------------------------------------------------
CC
C      CALL NLS(X,Y,YPRIME,NEQ,
C     *   RES,JAC,PSOL,H,WT,JSTART,IDID,RPAR,IPAR,PHI,GAMMA,
C     *   SAVR,DELTA,E,WM,IWM,CJ,CJOLD,CJLAST,S,
C     *   UROUND,EPLI,SQRTN,RSQRTN,EPCON,JCALC,JFLG,KP1,
C     *   NONNEG,NTYPE,IERNLS)
CC
C      IF(IERNLS .NE. 0)GO TO 600
CC
CC
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 4
CC     Estimate the errors at orders K,K-1,K-2
CC     as if constant stepsize was used. Estimate
CC     the local error at order K and test
CC     whether the current step is successful.
CC-----------------------------------------------------------------------
CC
CC     Estimate errors at orders K,K-1,K-2
C
CC    compute VT(H) 
CC ADDED LINES BELOW:  HR 24.07.2008
C      IF (INFO16 .EQ. 2) THEN
C        CALL DCALCVT(NEQ,H,INFO2,RTOL,ATOL,Y,VT,NRIND1, NRIND2, NRIND3,
C     *         RPAR, IPAR)
C        CALL DINVWT(NEQ,VT,IER)
C      IF (IER .NE. 0) THEN 
C        IDID = -2
C        GO TO 675
C      ENDIF 
C      ENDIF
C
C
C      ENORM = DDWNRM(NEQ,E,VT,RPAR,IPAR)
C      ERK = SIGMA(K+1)*ENORM
C      TERK = (K+1)*ERK
C      EST = ERK
C      KNEW=K
C      IF(K .EQ. 1)GO TO 430
C      DO 405 I = 1,NEQ
C405     DELTA(I) = PHI(I,KP1) + E(I)
C      ERKM1=SIGMA(K)*DDWNRM(NEQ,DELTA,VT,RPAR,IPAR)
C      TERKM1 = K*ERKM1
C      IF(K .GT. 2)GO TO 410
C      IF(TERKM1 .LE. 0.5*TERK)GO TO 420
C      GO TO 430
C410   CONTINUE
C      DO 415 I = 1,NEQ
C415     DELTA(I) = PHI(I,K) + DELTA(I)
C      ERKM2=SIGMA(K-1)*DDWNRM(NEQ,DELTA,VT,RPAR,IPAR)
C      TERKM2 = (K-1)*ERKM2
C      IF(MAX(TERKM1,TERKM2).GT.TERK)GO TO 430
CC
CC     Lower the order
CC
C420   CONTINUE
C      KNEW=K-1
C      EST = ERKM1
CC
CC
CC     Calculate the local error for the current step
CC     to see if the step was successful
CC
C430   CONTINUE
C      ERR = CK * ENORM
C      IF(ERR .GT. 1.0D0)GO TO 600
CC
CC
CC
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 5
CC     The step is successful. Determine
CC     the best order and stepsize for
CC     the next step. Update the differences
CC     for the next step.
CC-----------------------------------------------------------------------
C      IDID=1
C      IWM(LNST)=IWM(LNST)+1
C      KDIFF=K-KOLD
C      KOLD=K
C      HOLD=H
CC
CC
CC     Estimate the error at order K+1 unless
CC        already decided to lower order, or
CC        already using maximum order, or
CC        stepsize not constant, or
CC        order raised in previous step
CC
C      IF(KNEW.EQ.KM1.OR.K.EQ.IWM(LMXORD))IPHASE=1
C      IF(IPHASE .EQ. 0)GO TO 545
C      IF(KNEW.EQ.KM1)GO TO 540
C      IF(K.EQ.IWM(LMXORD)) GO TO 550
C      IF(KP1.GE.NS.OR.KDIFF.EQ.1)GO TO 550
C      DO 510 I=1,NEQ
C510      DELTA(I)=E(I)-PHI(I,KP2)
C      ERKP1 = (1.0D0/(K+2))*DDWNRM(NEQ,DELTA,VT,RPAR,IPAR)
C      TERKP1 = (K+2)*ERKP1
C      IF(K.GT.1)GO TO 520
C      IF(TERKP1.GE.0.5D0*TERK)GO TO 550
C      GO TO 530
C520   IF(TERKM1.LE.MIN(TERK,TERKP1))GO TO 540
C      IF(TERKP1.GE.TERK.OR.K.EQ.IWM(LMXORD))GO TO 550
CC
CC     Raise order
CC
C530   K=KP1
C      EST = ERKP1
C      GO TO 550
CC
CC     Lower order
CC
C540   K=KM1
C      EST = ERKM1
C      GO TO 550
CC
CC     If IPHASE = 0, increase order by one and multiply stepsize by
CC     factor two
CC
C545   K = KP1
C      HNEW = H*2.0D0
C      H = HNEW
C      GO TO 575
CC
CC
CC     Determine the appropriate stepsize for
CC     the next step.
CC
C550   HNEW=H
C      TEMP2=K+1
C      R=(2.0D0*EST+0.0001D0)**(-1.0D0/TEMP2)
C      IF(R .LT. 2.0D0) GO TO 555
C      HNEW = 2.0D0*H
C      GO TO 560
C555   IF(R .GT. 1.0D0) GO TO 560
C      R = MAX(0.5D0,MIN(0.9D0,R))
C      HNEW = H*R
C560   H=HNEW
CC
CC
CC     Update differences for next step
CC
C575   CONTINUE
C      IF(KOLD.EQ.IWM(LMXORD))GO TO 585
C      DO 580 I=1,NEQ
C580      PHI(I,KP2)=E(I)
C585   CONTINUE
C      DO 590 I=1,NEQ
C590      PHI(I,KP1)=PHI(I,KP1)+E(I)
C      DO 595 J1=2,KP1
C         J=KP1-J1+1
C         DO 595 I=1,NEQ
C595      PHI(I,J)=PHI(I,J)+PHI(I,J+1)
C      JSTART = 1
C      RETURN
CC
CC
CC
CC
CC
CC-----------------------------------------------------------------------
CC     BLOCK 6
CC     The step is unsuccessful. Restore X,PSI,PHI
CC     Determine appropriate stepsize for
CC     continuing the integration, or exit with
CC     an error flag if there have been many
CC     failures.
CC-----------------------------------------------------------------------
C600   IPHASE = 1
CC
CC     Restore X,PHI,PSI
CC
C      X=XOLD
C      IF(KP1.LT.NSP1)GO TO 630
C      DO 620 J=NSP1,KP1
C         TEMP1=1.0D0/BETA(J)
C         DO 610 I=1,NEQ
C610         PHI(I,J)=TEMP1*PHI(I,J)
C620      CONTINUE
C630   CONTINUE
C      DO 640 I=2,KP1
C640      PSI(I-1)=PSI(I)-H
CC
CC
CC     Test whether failure is due to nonlinear solver
CC     or error test
CC
C      IF(IERNLS .EQ. 0)GO TO 660
C      IWM(LCFN)=IWM(LCFN)+1
CC
CC
CC     The nonlinear solver failed to converge.
CC     Determine the cause of the failure and take appropriate action.
CC     If IERNLS .LT. 0, then return.  Otherwise, reduce the stepsize
CC     and try again, unless too many failures have occurred.
CC
C      IF (IERNLS .LT. 0) GO TO 675
C      NCF = NCF + 1
C      R = 0.25D0
C      H = H*R
C      IF (NCF .LT. 10 .AND. ABS(H) .GE. HMIN) GO TO 690
C      IF (IDID .EQ. 1) IDID = -7
C      IF (NEF .GE. 3) IDID = -9
C      GO TO 675
CC
CC
CC     The nonlinear solver converged, and the cause
CC     of the failure was the error estimate
CC     exceeding the tolerance.
CC
C660   NEF=NEF+1
C      IWM(LETF)=IWM(LETF)+1
C      IF (NEF .GT. 1) GO TO 665
CC
CC     On first error test failure, keep current order or lower
CC     order by one.  Compute new stepsize based on differences
CC     of the solution.
CC
C      K = KNEW
C      TEMP2 = K + 1
C      R = 0.90D0*(2.0D0*EST+0.0001D0)**(-1.0D0/TEMP2)
C      R = MAX(0.25D0,MIN(0.9D0,R))
C      H = H*R
C      IF (ABS(H) .GE. HMIN) GO TO 690
C      IDID = -6
C      GO TO 675
CC
CC     On second error test failure, use the current order or
CC     decrease order by one.  Reduce the stepsize by a factor of
CC     one quarter.
CC
C665   IF (NEF .GT. 2) GO TO 670
C      K = KNEW
C      R = 0.25D0
C      H = R*H
C      IF (ABS(H) .GE. HMIN) GO TO 690
C      IDID = -6
C      GO TO 675
CC
CC     On third and subsequent error test failures, set the order to
CC     one, and reduce the stepsize by a factor of one quarter.
CC
C670   K = 1
C      R = 0.25D0
C      H = R*H
C      IF (ABS(H) .GE. HMIN) GO TO 690
C      IDID = -6
C      GO TO 675
CC
CC
CC
CC
CC     For all crashes, restore Y to its last value,
CC     interpolate to find YPRIME at last X, and return.
CC
CC     Before returning, verify that the user has not set
CC     IDID to a nonnegative value.  If the user has set IDID
CC     to a nonnegative value, then reset IDID to be -7, indicating
CC     a failure in the nonlinear system solver.
CC
C675   CONTINUE
C      CALL DDATRP(X,X,Y,YPRIME,NEQ,K,PHI,PSI)
C      JSTART = 1
C      IF (IDID .GE. 0) IDID = -7
C      RETURN
CC
CC
CC     Go back and try this step again.  
CC     If this is the first step, reset PSI(1) and rescale PHI(*,2).
CC
C690   IF (KOLD .EQ. 0) THEN
C        PSI(1) = H
C        DO 695 I = 1,NEQ
C695       PHI(I,2) = R*PHI(I,2)
C        ENDIF
C      GO TO 200
CC
CC------END OF SUBROUTINE DDSTP------------------------------------------
C      END
C      SUBROUTINE DCNSTR (NEQ, Y, YNEW, ICNSTR, TAU, RLX, IRET, IVAR)
CC
CC***BEGIN PROLOGUE  DCNSTR
CC***DATE WRITTEN   950808   (YYMMDD)
CC***REVISION DATE  950814   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC This subroutine checks for constraint violations in the proposed 
CC new approximate solution YNEW.
CC If a constraint violation occurs, then a new step length, TAU,
CC is calculated, and this value is to be given to the linesearch routine
CC to calculate a new approximate solution YNEW.
CC
CC On entry:
CC
CC   NEQ    -- size of the nonlinear system, and the length of arrays
CC             Y, YNEW and ICNSTR.
CC
CC   Y      -- real array containing the current approximate y.
CC
CC   YNEW   -- real array containing the new approximate y.
CC
CC   ICNSTR -- INTEGER array of length NEQ containing flags indicating
CC             which entries in YNEW are to be constrained.
CC             if ICNSTR(I) =  2, then YNEW(I) must be .GT. 0,
CC             if ICNSTR(I) =  1, then YNEW(I) must be .GE. 0,
CC             if ICNSTR(I) = -1, then YNEW(I) must be .LE. 0, while
CC             if ICNSTR(I) = -2, then YNEW(I) must be .LT. 0, while
CC             if ICNSTR(I) =  0, then YNEW(I) is not constrained.
CC
CC   RLX    -- real scalar restricting update, if ICNSTR(I) = 2 or -2,
CC             to ABS( (YNEW-Y)/Y ) < FAC2*RLX in component I.
CC
CC   TAU    -- the current size of the step length for the linesearch.
CC
CC On return
CC
CC   TAU    -- the adjusted size of the step length if a constraint
CC             violation occurred (otherwise, it is unchanged).  it is
CC             the step length to give to the linesearch routine.
CC
CC   IRET   -- output flag.
CC             IRET=0 means that YNEW satisfied all constraints.
CC             IRET=1 means that YNEW failed to satisfy all the
CC                    constraints, and a new linesearch step
CC                    must be computed.
CC
CC   IVAR   -- index of variable causing constraint to be violated.
CC
CC-----------------------------------------------------------------------
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(NEQ), YNEW(NEQ), ICNSTR(NEQ)
C      SAVE FAC, FAC2, ZERO
C      DATA FAC /0.6D0/, FAC2 /0.9D0/, ZERO/0.0D0/
CC-----------------------------------------------------------------------
CC Check constraints for proposed new step YNEW.  If a constraint has
CC been violated, then calculate a new step length, TAU, to be
CC used in the linesearch routine.
CC-----------------------------------------------------------------------
C      IRET = 0
C      RDYMX = ZERO
C      IVAR = 0
C      DO 100 I = 1,NEQ
CC
C         IF (ICNSTR(I) .EQ. 2) THEN
C            RDY = ABS( (YNEW(I)-Y(I))/Y(I) )
C            IF (RDY .GT. RDYMX) THEN
C               RDYMX = RDY
C               IVAR = I
C            ENDIF
C            IF (YNEW(I) .LE. ZERO) THEN
C               TAU = FAC*TAU
C               IVAR = I
C               IRET = 1
C               RETURN
C            ENDIF
CC
C         ELSEIF (ICNSTR(I) .EQ. 1) THEN
C            IF (YNEW(I) .LT. ZERO) THEN
C               TAU = FAC*TAU
C               IVAR = I
C               IRET = 1
C               RETURN
C            ENDIF
CC
C         ELSEIF (ICNSTR(I) .EQ. -1) THEN
C            IF (YNEW(I) .GT. ZERO) THEN
C               TAU = FAC*TAU
C               IVAR = I
C               IRET = 1
C               RETURN
C            ENDIF
CC
C         ELSEIF (ICNSTR(I) .EQ. -2) THEN
C            RDY = ABS( (YNEW(I)-Y(I))/Y(I) )
C            IF (RDY .GT. RDYMX) THEN
C               RDYMX = RDY
C               IVAR = I
C            ENDIF
C            IF (YNEW(I) .GE. ZERO) THEN
C               TAU = FAC*TAU
C               IVAR = I
C               IRET = 1
C               RETURN
C            ENDIF
CC
C         ENDIF
C 100  CONTINUE
C
C      IF(RDYMX .GE. RLX) THEN
C         TAU = FAC2*TAU*RLX/RDYMX
C         IRET = 1
C      ENDIF
CC
C      RETURN
CC----------------------- END OF SUBROUTINE DCNSTR ----------------------
C      END
C      SUBROUTINE DCNST0 (NEQ, Y, ICNSTR, IRET)
CC
CC***BEGIN PROLOGUE  DCNST0
CC***DATE WRITTEN   950808   (YYMMDD)
CC***REVISION DATE  950808   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC This subroutine checks for constraint violations in the initial 
CC approximate solution u.
CC
CC On entry
CC
CC   NEQ    -- size of the nonlinear system, and the length of arrays
CC             Y and ICNSTR.
CC
CC   Y      -- real array containing the initial approximate root.
CC
CC   ICNSTR -- INTEGER array of length NEQ containing flags indicating
CC             which entries in Y are to be constrained.
CC             if ICNSTR(I) =  2, then Y(I) must be .GT. 0,
CC             if ICNSTR(I) =  1, then Y(I) must be .GE. 0,
CC             if ICNSTR(I) = -1, then Y(I) must be .LE. 0, while
CC             if ICNSTR(I) = -2, then Y(I) must be .LT. 0, while
CC             if ICNSTR(I) =  0, then Y(I) is not constrained.
CC
CC On return
CC
CC   IRET   -- output flag.
CC             IRET=0    means that u satisfied all constraints.
CC             IRET.NE.0 means that Y(IRET) failed to satisfy its
CC                       constraint.
CC
CC-----------------------------------------------------------------------
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(NEQ), ICNSTR(NEQ)
C      SAVE ZERO
C      DATA ZERO/0.D0/
CC-----------------------------------------------------------------------
CC Check constraints for initial Y.  If a constraint has been violated,
CC set IRET = I to signal an error return to calling routine.
CC-----------------------------------------------------------------------
C      IRET = 0
C      DO 100 I = 1,NEQ
C         IF (ICNSTR(I) .EQ. 2) THEN
C            IF (Y(I) .LE. ZERO) THEN
C               IRET = I
C               RETURN
C            ENDIF
C         ELSEIF (ICNSTR(I) .EQ. 1) THEN
C            IF (Y(I) .LT. ZERO) THEN
C               IRET = I
C               RETURN
C            ENDIF 
C         ELSEIF (ICNSTR(I) .EQ. -1) THEN
C            IF (Y(I) .GT. ZERO) THEN
C               IRET = I
C               RETURN
C            ENDIF 
C         ELSEIF (ICNSTR(I) .EQ. -2) THEN
C            IF (Y(I) .GE. ZERO) THEN
C               IRET = I
C               RETURN
C            ENDIF 
C        ENDIF
C 100  CONTINUE
C      RETURN
CC----------------------- END OF SUBROUTINE DCNST0 ----------------------
C      END
C      SUBROUTINE DDAWTS(NEQ,IWT,RTOL,ATOL,Y,WT,RPAR,IPAR)
CC
CC***BEGIN PROLOGUE  DDAWTS
CC***REFER TO  DDASPK
CC***ROUTINES CALLED  (NONE)
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***END PROLOGUE  DDAWTS
CC-----------------------------------------------------------------------
CC     This subroutine sets the error weight vector,
CC     WT, according to WT(I)=RTOL(I)*ABS(Y(I))+ATOL(I),
CC     I = 1 to NEQ.
CC     RTOL and ATOL are scalars if IWT = 0,
CC     and vectors if IWT = 1.
CC-----------------------------------------------------------------------
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION RTOL(*),ATOL(*),Y(*),WT(*)
C      DIMENSION RPAR(*),IPAR(*)
C      RTOLI=RTOL(1)
C      ATOLI=ATOL(1)
C      DO 20 I=1,NEQ
C         IF (IWT .EQ.0) GO TO 10
C           RTOLI=RTOL(I)
C           ATOLI=ATOL(I)
C10         WT(I)=RTOLI*ABS(Y(I))+ATOLI
C20         CONTINUE
C      RETURN
CC
CC------END OF SUBROUTINE DDAWTS-----------------------------------------
C      END 
C      SUBROUTINE DCALCVT(NEQ,HSCALE,IWT,RTOL,ATOL,Y,WT,NIND1,NIND2,
C     *   NIND3,RPAR,IPAR)
C
CC***BEGIN PROLOGUE  DCALCVT
CC***REFER TO  DDASPK
CC***ROUTINES CALLED  (NONE)
CC***DATE WRITTEN   080722   (YYMMDD)    
CC***END PROLOGUE  DCALCVT
CC-----------------------------------------------------------------------
CC     Description:
CC
CC     This subroutine sets the error weight vector, WT, for error test
CC     with stepsize scaled variables.
CC     It is usefull to sclae variables with the stepsize accrding to
CC     their index:
CC     index 1 varibales remain unscaled
CC     index 2 varibales are scaled with stepsize HSCALE      
CC     index 3 varibales are scaled with HSCALE**2
CC     This means for the weight vector WT:
CC       WT(I)=RTOL(I)*ABS(Y(I))+ATOL(I)                  Y(I): index 1
CC       WT(I)=RTOL(I)*ABS(Y(I))+ATOL(I)/HSCAlE           Y(I): index 2
CC       WT(I)=RTOL(I)*ABS(Y(I))+ATOL(I)/HSCAlE/HSCAlE    Y(I): index 3
CC     See to Hairer, Lubich, Roche (1989): The numerical solution
CC     of differential algebraic systems by Runge-Kutta methods.
CC     Lecture Notes in Math. 1409, Springer for justification.
CC
CC     NEQ    : number of equations (dimension of Y)
CC     HSCALE : stepsize used as sclae factor
CC     IWT    : 0 scalar tolerances or 1 for vector tol. (=info(2))
CC     RTOL   : relative tolerances
CC     ATOL   : absolute tolerances
CC     Y      : solution
CC     WT     : weight vector (OUTPUT)
CC     NIND1  : number of index 1 variables
CC     NIND2  : number of index 2 variables
CC     NIND3  : number of index 3 variables
CC
CC  !!! ATTENTION !!      
CC       THE RES FUNCTION-SUBROUTINE SHOULD BE WRITTEN SUCH THAT
CC       THE INDEX 1,2,3 VARIABLES APPEAR IN THIS ORDER.      
CC-----------------------------------------------------------------------
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION RTOL(*),ATOL(*),Y(*),WT(*)
C      DIMENSION RPAR(*),IPAR(*)
C
C      RTOLI=RTOL(1)
C      ATOLI=ATOL(1)
C      DO 20 I=1,NIND1
C         IF (IWT .EQ.0) GO TO 10
C           RTOLI=RTOL(I)
C           ATOLI=ATOL(I)
C10         WT(I)=RTOLI*ABS(Y(I))+ATOLI  
C20         CONTINUE
C      DO 40 I=1,NIND2
C         II= NIND1+I
C         IF (IWT .EQ.0) GO TO 30
C           RTOLI=RTOL(II)
C           ATOLI=ATOL(II)
C30         WT(II)=RTOLI*ABS(Y(II))+ATOLI/HSCALE  
C40         CONTINUE
C      DO 60 I=1,NIND3
C         II= NIND1+NIND2+I
C         IF (IWT .EQ.0) GO TO 50
C           RTOLI=RTOL(II)
C           ATOLI=ATOL(II)
C50         WT(II)=RTOLI*ABS(Y(II))+ATOLI/HSCALE/HSCALE
C60         CONTINUE
C      RETURN
CC
CC------END OF SUBROUTINE DCALCVT-----------------------------------------
C      END
C      SUBROUTINE DINVWT(NEQ,WT,IER)
CC
CC***BEGIN PROLOGUE  DINVWT
CC***REFER TO  DDASPK
CC***ROUTINES CALLED  (NONE)
CC***DATE WRITTEN   950125   (YYMMDD)
CC***END PROLOGUE  DINVWT
CC-----------------------------------------------------------------------
CC     This subroutine checks the error weight vector WT, of length NEQ,
CC     for components that are .le. 0, and if none are found, it
CC     inverts the WT(I) in place.  This replaces division operations
CC     with multiplications in all norm evaluations.
CC     IER is returned as 0 if all WT(I) were found positive,
CC     and the first I with WT(I) .le. 0.0 otherwise.
CC-----------------------------------------------------------------------
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION WT(*)
CC
C      DO 10 I = 1,NEQ
C        IF (WT(I) .LE. 0.0D0) GO TO 30
C 10     CONTINUE
C      DO 20 I = 1,NEQ
C 20     WT(I) = 1.0D0/WT(I)
C      IER = 0
C      RETURN
CC
C 30   IER = I
C      RETURN
CC
CC------END OF SUBROUTINE DINVWT-----------------------------------------
C      END
C      SUBROUTINE DDATRP(X,XOUT,YOUT,YPOUT,NEQ,KOLD,PHI,PSI)
CC
CC***BEGIN PROLOGUE  DDATRP
CC***REFER TO  DDASPK
CC***ROUTINES CALLED  (NONE)
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***END PROLOGUE  DDATRP
CC
CC-----------------------------------------------------------------------
CC     The methods in subroutine DDSTP use polynomials
CC     to approximate the solution.  DDATRP approximates the
CC     solution and its derivative at time XOUT by evaluating
CC     one of these polynomials, and its derivative, there.
CC     Information defining this polynomial is passed from
CC     DDSTP, so DDATRP cannot be used alone.
CC
CC     The parameters are
CC
CC     X     The current time in the integration.
CC     XOUT  The time at which the solution is desired.
CC     YOUT  The interpolated approximation to Y at XOUT.
CC           (This is output.)
CC     YPOUT The interpolated approximation to YPRIME at XOUT.
CC           (This is output.)
CC     NEQ   Number of equations.
CC     KOLD  Order used on last successful step.
CC     PHI   Array of scaled divided differences of Y.
CC     PSI   Array of past stepsize history.
CC-----------------------------------------------------------------------
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION YOUT(*),YPOUT(*)
C      DIMENSION PHI(NEQ,*),PSI(*)
C      KOLDP1=KOLD+1
C      TEMP1=XOUT-X
C      DO 10 I=1,NEQ
C         YOUT(I)=PHI(I,1)
C10       YPOUT(I)=0.0D0
C      C=1.0D0
C      D=0.0D0
C      GAMMA=TEMP1/PSI(1)
C      DO 30 J=2,KOLDP1
C         D=D*GAMMA+C/PSI(J-1)
C         C=C*GAMMA
C         GAMMA=(TEMP1+PSI(J-1))/PSI(J)
C         DO 20 I=1,NEQ
C            YOUT(I)=YOUT(I)+C*PHI(I,J)
C20          YPOUT(I)=YPOUT(I)+D*PHI(I,J)
C30       CONTINUE
C      RETURN
CC
CC------END OF SUBROUTINE DDATRP-----------------------------------------
C      END
C      DOUBLE PRECISION FUNCTION DDWNRM(NEQ,V,RWT,RPAR,IPAR)
CC
CC***BEGIN PROLOGUE  DDWNRM
CC***ROUTINES CALLED  (NONE)
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***END PROLOGUE  DDWNRM
CC-----------------------------------------------------------------------
CC     This function routine computes the weighted
CC     root-mean-square norm of the vector of length
CC     NEQ contained in the array V, with reciprocal weights
CC     contained in the array RWT of length NEQ.
CC        DDWNRM=SQRT((1/NEQ)*SUM(V(I)*RWT(I))**2)
CC-----------------------------------------------------------------------
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION V(*),RWT(*)
C      DIMENSION RPAR(*),IPAR(*)
C      DDWNRM = 0.0D0
C      VMAX = 0.0D0
C      DO 10 I = 1,NEQ
C        IF(ABS(V(I)*RWT(I)) .GT. VMAX) VMAX = ABS(V(I)*RWT(I))
C10    CONTINUE
C      IF(VMAX .LE. 0.0D0) GO TO 30
C      SUM = 0.0D0
C      DO 20 I = 1,NEQ
C20      SUM = SUM + ((V(I)*RWT(I))/VMAX)**2
C      DDWNRM = VMAX*SQRT(SUM/NEQ)
C30    CONTINUE
C      RETURN
CC
CC------END OF FUNCTION DDWNRM-------------------------------------------
C      END
C      SUBROUTINE DDASID(X,Y,YPRIME,NEQ,ICOPT,ID,RES,JACD,PDUM,H,TSCALE,
C     *  WT,JSDUM,RPAR,IPAR,DUMSVR,DELTA,R,YIC,YPIC,DUMPWK,WM,IWM,CJ,
C     *  UROUND,DUME,DUMS,DUMR,EPCON,RATEMX,STPTOL,JFDUM,
C     *  ICNFLG,ICNSTR,IERNLS)
CC
CC***BEGIN PROLOGUE  DDASID
CC***REFER TO  DDASPK
CC***DATE WRITTEN   940701   (YYMMDD)
CC***REVISION DATE  950808   (YYMMDD)
CC***REVISION DATE  951110   Removed unreachable block 390.
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC
CC     DDASID solves a nonlinear system of algebraic equations of the
CC     form G(X,Y,YPRIME) = 0 for the unknown parts of Y and YPRIME in
CC     the initial conditions.
CC
CC     The method used is a modified Newton scheme.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     ICOPT     -- Initial condition option chosen (1 or 2).
CC     ID        -- Array of dimension NEQ, which must be initialized
CC                  if ICOPT = 1.  See DDASIC.
CC     RES       -- External user-supplied subroutine to evaluate the
CC                  residual.  See RES description in DDASPK prologue.
CC     JACD      -- External user-supplied routine to evaluate the
CC                  Jacobian.  See JAC description for the case
CC                  INFO(12) = 0 in the DDASPK prologue.
CC     PDUM      -- Dummy argument.
CC     H         -- Scaling factor for this initial condition calc.
CC     TSCALE    -- Scale factor in T, used for stopping tests if nonzero.
CC     WT        -- Vector of weights for error criterion.
CC     JSDUM     -- Dummy argument.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     DUMSVR    -- Dummy argument.
CC     DELTA     -- Work vector for NLS of length NEQ.
CC     R         -- Work vector for NLS of length NEQ.
CC     YIC,YPIC  -- Work vectors for NLS, each of length NEQ.
CC     DUMPWK    -- Dummy argument.
CC     WM,IWM    -- Real and integer arrays storing matrix information
CC                  such as the matrix of partial derivatives,
CC                  permutation vector, and various other information.
CC     CJ        -- Matrix parameter = 1/H (ICOPT = 1) or 0 (ICOPT = 2).
CC     UROUND    -- Unit roundoff.
CC     DUME      -- Dummy argument.
CC     DUMS      -- Dummy argument.
CC     DUMR      -- Dummy argument.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     RATEMX    -- Maximum convergence rate for which Newton iteration
CC                  is considered converging.
CC     JFDUM     -- Dummy argument.
CC     STPTOL    -- Tolerance used in calculating the minimum lambda
CC                  value allowed.
CC     ICNFLG    -- Integer scalar.  If nonzero, then constraint
CC                  violations in the proposed new approximate solution
CC                  will be checked for, and the maximum step length 
CC                  will be adjusted accordingly.
CC     ICNSTR    -- Integer array of length NEQ containing flags for
CC                  checking constraints.
CC     IERNLS    -- Error flag for nonlinear solver.
CC                   0   ==> nonlinear solver converged.
CC                   1,2 ==> recoverable error inside nonlinear solver.
CC                           1 => retry with current Y, YPRIME
CC                           2 => retry with original Y, YPRIME
CC                  -1   ==> unrecoverable error in nonlinear solver.
CC
CC     All variables with "DUM" in their names are dummy variables
CC     which are not used in this routine.
CC
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   RES, DMATD, DNSID
CC
CC***END PROLOGUE  DDASID
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),ID(*),WT(*),ICNSTR(*)
C      DIMENSION DELTA(*),R(*),YIC(*),YPIC(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      EXTERNAL  RES, JACD
CC
C      PARAMETER (LNRE=12, LNJE=13, LMXNIT=32, LMXNJ=33)
CC
CC
CC     Perform initializations.
CC
C      MXNIT = IWM(LMXNIT)
C      MXNJ = IWM(LMXNJ)
C      IERNLS = 0
C      NJ = 0
CC
CC     Call RES to initialize DELTA.
CC
C      IRES = 0
C      IWM(LNRE) = IWM(LNRE) + 1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 370
CC
CC     Looping point for updating the Jacobian.
CC
C300   CONTINUE
CC
CC     Initialize all error flags to zero.
CC
C      IERJ = 0
C      IRES = 0
C      IERNEW = 0
CC
CC     Reevaluate the iteration matrix, J = dG/dY + CJ*dG/dYPRIME,
CC     where G(X,Y,YPRIME) = 0.
CC
C      NJ = NJ + 1
C      IWM(LNJE)=IWM(LNJE)+1
C      CALL DMATD(NEQ,X,Y,YPRIME,DELTA,CJ,H,IERJ,WT,R,
C     *              WM,IWM,RES,IRES,UROUND,JACD,RPAR,IPAR)
C      IF (IRES .LT. 0 .OR. IERJ .NE. 0) GO TO 370
CC
CC     Call the nonlinear Newton solver for up to MXNIT iterations.
CC
C      CALL DNSID(X,Y,YPRIME,NEQ,ICOPT,ID,RES,WT,RPAR,IPAR,DELTA,R,
C     *     YIC,YPIC,WM,IWM,CJ,TSCALE,EPCON,RATEMX,MXNIT,STPTOL,
C     *     ICNFLG,ICNSTR,IERNEW)
CC
C      IF (IERNEW .EQ. 1 .AND. NJ .LT. MXNJ) THEN
CC
CC        MXNIT iterations were done, the convergence rate is < 1,
CC        and the number of Jacobian evaluations is less than MXNJ.
CC        Call RES, reevaluate the Jacobian, and try again.
CC
C         IWM(LNRE)=IWM(LNRE)+1
C         CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C         IF (IRES .LT. 0) GO TO 370
C         GO TO 300
C         ENDIF
CC
C      IF (IERNEW .NE. 0) GO TO 380
C
C      RETURN
CC
CC
CC     Unsuccessful exits from nonlinear solver.
CC     Compute IERNLS accordingly.
CC
C370   IERNLS = 2
C      IF (IRES .LE. -2) IERNLS = -1
C      RETURN
CC
C380   IERNLS = MIN(IERNEW,2)
C      RETURN
CC
CC------END OF SUBROUTINE DDASID-----------------------------------------
C      END
C      SUBROUTINE DNSID(X,Y,YPRIME,NEQ,ICOPT,ID,RES,WT,RPAR,IPAR,
C     *   DELTA,R,YIC,YPIC,WM,IWM,CJ,TSCALE,EPCON,RATEMX,MAXIT,STPTOL,
C     *   ICNFLG,ICNSTR,IERNEW)
CC
CC***BEGIN PROLOGUE  DNSID
CC***REFER TO  DDASPK
CC***DATE WRITTEN   940701   (YYMMDD)
CC***REVISION DATE  950713   (YYMMDD)
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNSID solves a nonlinear system of algebraic equations of the
CC     form G(X,Y,YPRIME) = 0 for the unknown parts of Y and YPRIME
CC     in the initial conditions.
CC
CC     The method used is a modified Newton scheme.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     ICOPT     -- Initial condition option chosen (1 or 2).
CC     ID        -- Array of dimension NEQ, which must be initialized
CC                  if ICOPT = 1.  See DDASIC.
CC     RES       -- External user-supplied subroutine to evaluate the
CC                  residual.  See RES description in DDASPK prologue.
CC     WT        -- Vector of weights for error criterion.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     DELTA     -- Residual vector on entry, and work vector of
CC                  length NEQ for DNSID.
CC     WM,IWM    -- Real and integer arrays storing matrix information
CC                  such as the matrix of partial derivatives,
CC                  permutation vector, and various other information.
CC     CJ        -- Matrix parameter = 1/H (ICOPT = 1) or 0 (ICOPT = 2).
CC     TSCALE    -- Scale factor in T, used for stopping tests if nonzero.
CC     R         -- Array of length NEQ used as workspace by the 
CC                  linesearch routine DLINSD.
CC     YIC,YPIC  -- Work vectors for DLINSD, each of length NEQ.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     RATEMX    -- Maximum convergence rate for which Newton iteration
CC                  is considered converging.
CC     MAXIT     -- Maximum allowed number of Newton iterations.
CC     STPTOL    -- Tolerance used in calculating the minimum lambda
CC                  value allowed.
CC     ICNFLG    -- Integer scalar.  If nonzero, then constraint
CC                  violations in the proposed new approximate solution
CC                  will be checked for, and the maximum step length 
CC                  will be adjusted accordingly.
CC     ICNSTR    -- Integer array of length NEQ containing flags for
CC                  checking constraints.
CC     IERNEW    -- Error flag for Newton iteration.
CC                   0  ==> Newton iteration converged.
CC                   1  ==> failed to converge, but RATE .le. RATEMX.
CC                   2  ==> failed to converge, RATE .gt. RATEMX.
CC                   3  ==> other recoverable error (IRES = -1, or
CC                          linesearch failed).
CC                  -1  ==> unrecoverable error (IRES = -2).
CC
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   DSLVD, DDWNRM, DLINSD, DCOPY
CC
CC***END PROLOGUE  DNSID
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*),R(*)
C      DIMENSION ID(*),DELTA(*), YIC(*), YPIC(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      DIMENSION ICNSTR(*)
C      EXTERNAL  RES
CC
C      PARAMETER (LNNI=19, LLSOFF=35)
CC
CC
CC     Initializations.  M is the Newton iteration counter.
CC
C      LSOFF = IWM(LLSOFF)
C      M = 0
C      RATE = 1.0D0
C      RLX = 0.4D0
CC
CC     Compute a new step vector DELTA by back-substitution.
CC
C      CALL DSLVD (NEQ, DELTA, WM, IWM)
CC
CC     Get norm of DELTA.  Return now if norm(DELTA) .le. EPCON.
CC
C      DELNRM = DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      FNRM = DELNRM
C      IF (TSCALE .GT. 0.0D0) FNRM = FNRM*TSCALE*ABS(CJ)
C      IF (FNRM .LE. EPCON) RETURN
CC
CC     Newton iteration loop.
CC
C 300  CONTINUE
C      IWM(LNNI) = IWM(LNNI) + 1
CC
CC     Call linesearch routine for global strategy and set RATE
CC
C      OLDFNM = FNRM
CC
C      CALL DLINSD (NEQ, Y, X, YPRIME, CJ, TSCALE, DELTA, DELNRM, WT,
C     *             LSOFF, STPTOL, IRET, RES, IRES, WM, IWM, FNRM, ICOPT,
C     *             ID, R, YIC, YPIC, ICNFLG, ICNSTR, RLX, RPAR, IPAR)
CC
C      RATE = FNRM/OLDFNM
CC
CC     Check for error condition from linesearch.
C      IF (IRET .NE. 0) GO TO 390
CC
CC     Test for convergence of the iteration, and return or loop.
CC
C      IF (FNRM .LE. EPCON) RETURN
CC
CC     The iteration has not yet converged.  Update M.
CC     Test whether the maximum number of iterations have been tried.
CC
C      M = M + 1
C      IF (M .GE. MAXIT) GO TO 380
CC
CC     Copy the residual to DELTA and its norm to DELNRM, and loop for
CC     another iteration.
CC
C      CALL DCOPY (NEQ, R, 1, DELTA, 1)
C      DELNRM = FNRM      
C      GO TO 300
CC
CC     The maximum number of iterations was done.  Set IERNEW and return.
CC
C 380  IF (RATE .LE. RATEMX) THEN
C         IERNEW = 1
C      ELSE
C         IERNEW = 2
C      ENDIF
C      RETURN
CC
C 390  IF (IRES .LE. -2) THEN
C         IERNEW = -1
C      ELSE
C         IERNEW = 3
C      ENDIF
C      RETURN
CC
CC
CC------END OF SUBROUTINE DNSID------------------------------------------
C      END
C      SUBROUTINE DLINSD (NEQ, Y, T, YPRIME, CJ, TSCALE, P, PNRM, WT,
C     *                   LSOFF, STPTOL, IRET, RES, IRES, WM, IWM,
C     *                   FNRM, ICOPT, ID, R, YNEW, YPNEW, ICNFLG,
C     *                   ICNSTR, RLX, RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DLINSD
CC***REFER TO  DNSID
CC***DATE WRITTEN   941025   (YYMMDD)
CC***REVISION DATE  941215   (YYMMDD)
CC***REVISION DATE  960129   Moved line RL = ONE to top block.
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DLINSD uses a linesearch algorithm to calculate a new (Y,YPRIME)
CC     pair (YNEW,YPNEW) such that 
CC
CC     f(YNEW,YPNEW) .le. (1 - 2*ALPHA*RL)*f(Y,YPRIME) ,
CC
CC     where 0 < RL <= 1.  Here, f(y,y') is defined as
CC
CC      f(y,y') = (1/2)*norm( (J-inverse)*G(t,y,y') )**2 ,
CC
CC     where norm() is the weighted RMS vector norm, G is the DAE
CC     system residual function, and J is the system iteration matrix
CC     (Jacobian).
CC
CC     In addition to the parameters defined elsewhere, we have
CC
CC     TSCALE  --  Scale factor in T, used for stopping tests if nonzero.
CC     P       -- Approximate Newton step used in backtracking.
CC     PNRM    -- Weighted RMS norm of P.
CC     LSOFF   -- Flag showing whether the linesearch algorithm is
CC                to be invoked.  0 means do the linesearch, and
CC                1 means turn off linesearch.
CC     STPTOL  -- Tolerance used in calculating the minimum lambda
CC                value allowed.
CC     ICNFLG  -- Integer scalar.  If nonzero, then constraint violations
CC                in the proposed new approximate solution will be
CC                checked for, and the maximum step length will be
CC                adjusted accordingly.
CC     ICNSTR  -- Integer array of length NEQ containing flags for
CC                checking constraints.
CC     RLX     -- Real scalar restricting update size in DCNSTR.
CC     YNEW    -- Array of length NEQ used to hold the new Y in
CC                performing the linesearch.
CC     YPNEW   -- Array of length NEQ used to hold the new YPRIME in
CC                performing the linesearch.
CC     Y       -- Array of length NEQ containing the new Y (i.e.,=YNEW).
CC     YPRIME  -- Array of length NEQ containing the new YPRIME 
CC                (i.e.,=YPNEW).
CC     FNRM    -- Real scalar containing SQRT(2*f(Y,YPRIME)) for the
CC                current (Y,YPRIME) on input and output.
CC     R       -- Work array of length NEQ, containing the scaled 
CC                residual (J-inverse)*G(t,y,y') on return.
CC     IRET    -- Return flag.
CC                IRET=0 means that a satisfactory (Y,YPRIME) was found.
CC                IRET=1 means that the routine failed to find a new
CC                       (Y,YPRIME) that was sufficiently distinct from
CC                       the current (Y,YPRIME) pair.
CC                IRET=2 means IRES .ne. 0 from RES.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   DFNRMD, DYYPNW, DCNSTR, DCOPY, XERRWD
CC
CC***END PROLOGUE  DLINSD
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      EXTERNAL  RES
C      DIMENSION Y(*), YPRIME(*), WT(*), R(*), ID(*)
C      DIMENSION WM(*), IWM(*)
C      DIMENSION YNEW(*), YPNEW(*), P(*), ICNSTR(*)
C      DIMENSION RPAR(*), IPAR(*)
C      CHARACTER MSG*80
CC
C      PARAMETER (LNRE=12, LKPRIN=31)
CC
C      SAVE ALPHA, ONE, TWO
C      DATA ALPHA/1.0D-4/, ONE/1.0D0/, TWO/2.0D0/
CC
C      KPRIN=IWM(LKPRIN)
CC
C      F1NRM = (FNRM*FNRM)/TWO
C      RATIO = ONE
C      IF (KPRIN .GE. 2) THEN
C        MSG = '------ IN ROUTINE DLINSD-- PNRM = (R1)'
C        CALL XERRWD(MSG, 38, 901, 0, 0, 0, 0, 1, PNRM, 0.0D0)
C        ENDIF
C      TAU = PNRM
C      RL = ONE
CC-----------------------------------------------------------------------
CC Check for violations of the constraints, if any are imposed.
CC If any violations are found, the step vector P is rescaled, and the 
CC constraint check is repeated, until no violations are found.
CC-----------------------------------------------------------------------
C      IF (ICNFLG .NE. 0) THEN
C 10      CONTINUE
C         CALL DYYPNW (NEQ,Y,YPRIME,CJ,RL,P,ICOPT,ID,YNEW,YPNEW)
C         CALL DCNSTR (NEQ, Y, YNEW, ICNSTR, TAU, RLX, IRET, IVAR)
C         IF (IRET .EQ. 1) THEN
C            RATIO1 = TAU/PNRM
C            RATIO = RATIO*RATIO1
C            DO 20 I = 1,NEQ
C 20           P(I) = P(I)*RATIO1
C            PNRM = TAU
C            IF (KPRIN .GE. 2) THEN
C              MSG = '------ CONSTRAINT VIOL., PNRM = (R1), INDEX = (I1)'
C              CALL XERRWD(MSG, 50, 902, 0, 1, IVAR, 0, 1, PNRM, 0.0D0)
C              ENDIF
C            IF (PNRM .LE. STPTOL) THEN
C              IRET = 1
C              RETURN
C              ENDIF
C            GO TO 10
C            ENDIF
C         ENDIF
CC
C      SLPI = (-TWO*F1NRM)*RATIO
C      RLMIN = STPTOL/PNRM
C      IF (LSOFF .EQ. 0 .AND. KPRIN .GE. 2) THEN
C        MSG = '------ MIN. LAMBDA = (R1)'
C        CALL XERRWD(MSG, 25, 903, 0, 0, 0, 0, 1, RLMIN, 0.0D0)
C        ENDIF
CC-----------------------------------------------------------------------
CC Begin iteration to find RL value satisfying alpha-condition.
CC If RL becomes less than RLMIN, then terminate with IRET = 1.
CC-----------------------------------------------------------------------
C 100  CONTINUE
C      CALL DYYPNW (NEQ,Y,YPRIME,CJ,RL,P,ICOPT,ID,YNEW,YPNEW)
C      CALL DFNRMD (NEQ, YNEW, T, YPNEW, R, CJ, TSCALE, WT, RES, IRES,
C     *              FNRMP, WM, IWM, RPAR, IPAR)
C      IWM(LNRE) = IWM(LNRE) + 1
C      IF (IRES .NE. 0) THEN
C        IRET = 2
C        RETURN
C        ENDIF
C      IF (LSOFF .EQ. 1) GO TO 150
CC
C      F1NRMP = FNRMP*FNRMP/TWO
C      IF (KPRIN .GE. 2) THEN
C        MSG = '------ LAMBDA = (R1)'
C        CALL XERRWD(MSG, 20, 904, 0, 0, 0, 0, 1, RL, 0.0D0)
C        MSG = '------ NORM(F1) = (R1),  NORM(F1NEW) = (R2)'
C        CALL XERRWD(MSG, 43, 905, 0, 0, 0, 0, 2, F1NRM, F1NRMP)
C        ENDIF
C      IF (F1NRMP .GT. F1NRM + ALPHA*SLPI*RL) GO TO 200
CC-----------------------------------------------------------------------
CC Alpha-condition is satisfied, or linesearch is turned off.
CC Copy YNEW,YPNEW to Y,YPRIME and return.
CC-----------------------------------------------------------------------
C 150  IRET = 0
C      CALL DCOPY (NEQ, YNEW, 1, Y, 1)
C      CALL DCOPY (NEQ, YPNEW, 1, YPRIME, 1)
C      FNRM = FNRMP
C      IF (KPRIN .GE. 1) THEN
C        MSG = '------ LEAVING ROUTINE DLINSD, FNRM = (R1)'
C        CALL XERRWD(MSG, 42, 906, 0, 0, 0, 0, 1, FNRM, 0.0D0)
C        ENDIF
C      RETURN
CC-----------------------------------------------------------------------
CC Alpha-condition not satisfied.  Perform backtrack to compute new RL
CC value.  If no satisfactory YNEW,YPNEW can be found sufficiently 
CC distinct from Y,YPRIME, then return IRET = 1.
CC-----------------------------------------------------------------------
C 200  CONTINUE
C      IF (RL .LT. RLMIN) THEN
C        IRET = 1
C        RETURN
C        ENDIF
CC
C      RL = RL/TWO
C      GO TO 100
CC
CC----------------------- END OF SUBROUTINE DLINSD ----------------------
C      END
C      SUBROUTINE DFNRMD (NEQ, Y, T, YPRIME, R, CJ, TSCALE, WT,
C     *                   RES, IRES, FNORM, WM, IWM, RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DFNRMD
CC***REFER TO  DLINSD
CC***DATE WRITTEN   941025   (YYMMDD)
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DFNRMD calculates the scaled preconditioned norm of the nonlinear
CC     function used in the nonlinear iteration for obtaining consistent
CC     initial conditions.  Specifically, DFNRMD calculates the weighted
CC     root-mean-square norm of the vector (J-inverse)*G(T,Y,YPRIME),
CC     where J is the Jacobian matrix.
CC
CC     In addition to the parameters described in the calling program
CC     DLINSD, the parameters represent
CC
CC     R      -- Array of length NEQ that contains
CC               (J-inverse)*G(T,Y,YPRIME) on return.
CC     TSCALE -- Scale factor in T, used for stopping tests if nonzero.
CC     FNORM  -- Scalar containing the weighted norm of R on return.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   RES, DSLVD, DDWNRM
CC
CC***END PROLOGUE  DFNRMD
CC
CC
C      IMPLICIT DOUBLE PRECISION (A-H,O-Z)
C      EXTERNAL RES
C      DIMENSION Y(*), YPRIME(*), WT(*), R(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
CC-----------------------------------------------------------------------
CC     Call RES routine.
CC-----------------------------------------------------------------------
C      IRES = 0
C      CALL RES(T,Y,YPRIME,CJ,R,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) RETURN
CC-----------------------------------------------------------------------
CC     Apply inverse of Jacobian to vector R.
CC-----------------------------------------------------------------------
C      CALL DSLVD(NEQ,R,WM,IWM)
CC-----------------------------------------------------------------------
CC     Calculate norm of R.
CC-----------------------------------------------------------------------
C      FNORM = DDWNRM(NEQ,R,WT,RPAR,IPAR)
C      IF (TSCALE .GT. 0.0D0) FNORM = FNORM*TSCALE*ABS(CJ)
CC
C      RETURN
CC----------------------- END OF SUBROUTINE DFNRMD ----------------------
C      END
C      SUBROUTINE DNEDD(X,Y,YPRIME,NEQ,RES,JACD,PDUM,H,WT,
C     *   JSTART,IDID,RPAR,IPAR,PHI,GAMMA,DUMSVR,DELTA,E,
C     *   WM,IWM,CJ,CJOLD,CJLAST,S,UROUND,DUME,DUMS,DUMR,
C     *   EPCON,JCALC,JFDUM,KP1,NONNEG,NTYPE,IERNLS)
CC
CC***BEGIN PROLOGUE  DNEDD
CC***REFER TO  DDASPK
CC***DATE WRITTEN   891219   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNEDD solves a nonlinear system of
CC     algebraic equations of the form
CC     G(X,Y,YPRIME) = 0 for the unknown Y.
CC
CC     The method used is a modified Newton scheme.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     JACD      -- External user-supplied routine to evaluate the
CC                  Jacobian.  See JAC description for the case
CC                  INFO(12) = 0 in the DDASPK prologue.
CC     PDUM      -- Dummy argument.
CC     H         -- Appropriate step size for next step.
CC     WT        -- Vector of weights for error criterion.
CC     JSTART    -- Indicates first call to this routine.
CC                  If JSTART = 0, then this is the first call,
CC                  otherwise it is not.
CC     IDID      -- Completion flag, output by DNEDD.
CC                  See IDID description in DDASPK prologue.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     PHI       -- Array of divided differences used by
CC                  DNEDD.  The length is NEQ*(K+1),where
CC                  K is the maximum order.
CC     GAMMA     -- Array used to predict Y and YPRIME.  The length
CC                  is MAXORD+1 where MAXORD is the maximum order.
CC     DUMSVR    -- Dummy argument.
CC     DELTA     -- Work vector for NLS of length NEQ.
CC     E         -- Error accumulation vector for NLS of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information such as the matrix
CC                  of partial derivatives, permutation
CC                  vector, and various other information.
CC     CJ        -- Parameter always proportional to 1/H.
CC     CJOLD     -- Saves the value of CJ as of the last call to DMATD.
CC                  Accounts for changes in CJ needed to
CC                  decide whether to call DMATD.
CC     CJLAST    -- Previous value of CJ.
CC     S         -- A scalar determined by the approximate rate
CC                  of convergence of the Newton iteration and used
CC                  in the convergence test for the Newton iteration.
CC
CC                  If RATE is defined to be an estimate of the
CC                  rate of convergence of the Newton iteration,
CC                  then S = RATE/(1.D0-RATE).
CC
CC                  The closer RATE is to 0., the faster the Newton
CC                  iteration is converging; the closer RATE is to 1.,
CC                  the slower the Newton iteration is converging.
CC
CC                  On the first Newton iteration with an up-dated
CC                  preconditioner S = 100.D0, Thus the initial
CC                  RATE of convergence is approximately 1.
CC
CC                  S is preserved from call to call so that the rate
CC                  estimate from a previous step can be applied to
CC                  the current step.
CC     UROUND    -- Unit roundoff.
CC     DUME      -- Dummy argument.
CC     DUMS      -- Dummy argument.
CC     DUMR      -- Dummy argument.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     JCALC     -- Flag used to determine when to update
CC                  the Jacobian matrix.  In general:
CC
CC                  JCALC = -1 ==> Call the DMATD routine to update
CC                                 the Jacobian matrix.
CC                  JCALC =  0 ==> Jacobian matrix is up-to-date.
CC                  JCALC =  1 ==> Jacobian matrix is out-dated,
CC                                 but DMATD will not be called unless
CC                                 JCALC is set to -1.
CC     JFDUM     -- Dummy argument.
CC     KP1       -- The current order(K) + 1;  updated across calls.
CC     NONNEG    -- Flag to determine nonnegativity constraints.
CC     NTYPE     -- Identification code for the NLS routine.
CC                   0  ==> modified Newton; direct solver.
CC     IERNLS    -- Error flag for nonlinear solver.
CC                   0  ==> nonlinear solver converged.
CC                   1  ==> recoverable error inside nonlinear solver.
CC                  -1  ==> unrecoverable error inside nonlinear solver.
CC
CC     All variables with "DUM" in their names are dummy variables
CC     which are not used in this routine.
CC
CC     Following is a list and description of local variables which
CC     may not have an obvious usage.  They are listed in roughly the
CC     order they occur in this subroutine.
CC
CC     The following group of variables are passed as arguments to
CC     the Newton iteration solver.  They are explained in greater detail
CC     in DNSD:
CC        TOLNEW, MULDEL, MAXIT, IERNEW
CC
CC     IERTYP -- Flag which tells whether this subroutine is correct.
CC               0 ==> correct subroutine.
CC               1 ==> incorrect subroutine.
CC 
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   DDWNRM, RES, DMATD, DNSD
CC
CC***END PROLOGUE  DNEDD
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*)
C      DIMENSION DELTA(*),E(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      DIMENSION PHI(NEQ,*),GAMMA(*)
C      EXTERNAL  RES, JACD
CC
C      PARAMETER (LNRE=12, LNJE=13)
CC
C      SAVE MULDEL, MAXIT, XRATE
C      DATA MULDEL/1/, MAXIT/4/, XRATE/0.25D0/
CC
CC     Verify that this is the correct subroutine.
CC
C      IERTYP = 0
C      IF (NTYPE .NE. 0) THEN
C         IERTYP = 1
C         GO TO 380
C         ENDIF
CC
CC     If this is the first step, perform initializations.
CC
C      IF (JSTART .EQ. 0) THEN
C         CJOLD = CJ
C         JCALC = -1
C         ENDIF
CC
CC     Perform all other initializations.
CC
C      IERNLS = 0
CC
CC     Decide whether new Jacobian is needed.
CC
C      TEMP1 = (1.0D0 - XRATE)/(1.0D0 + XRATE)
C      TEMP2 = 1.0D0/TEMP1
C      IF (CJ/CJOLD .LT. TEMP1 .OR. CJ/CJOLD .GT. TEMP2) JCALC = -1
C      IF (CJ .NE. CJLAST) S = 100.D0
CC
CC-----------------------------------------------------------------------
CC     Entry point for updating the Jacobian with current
CC     stepsize.
CC-----------------------------------------------------------------------
C300   CONTINUE
CC
CC     Initialize all error flags to zero.
CC
C      IERJ = 0
C      IRES = 0
C      IERNEW = 0
CC
CC     Predict the solution and derivative and compute the tolerance
CC     for the Newton iteration.
CC
C      DO 310 I=1,NEQ
C         Y(I)=PHI(I,1)
C310      YPRIME(I)=0.0D0
C      DO 330 J=2,KP1
C         DO 320 I=1,NEQ
C            Y(I)=Y(I)+PHI(I,J)
C320         YPRIME(I)=YPRIME(I)+GAMMA(J)*PHI(I,J)
C330   CONTINUE
C      PNORM = DDWNRM (NEQ,Y,WT,RPAR,IPAR)
C      TOLNEW = 100.D0*UROUND*PNORM
CC     
CC     Call RES to initialize DELTA.
CC
C      IWM(LNRE)=IWM(LNRE)+1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 380
CC
CC     If indicated, reevaluate the iteration matrix 
CC     J = dG/dY + CJ*dG/dYPRIME (where G(X,Y,YPRIME)=0).
CC     Set JCALC to 0 as an indicator that this has been done.
CC
C      IF(JCALC .EQ. -1) THEN
C         IWM(LNJE)=IWM(LNJE)+1
C         JCALC=0
C         CALL DMATD(NEQ,X,Y,YPRIME,DELTA,CJ,H,IERJ,WT,E,WM,IWM,
C     *              RES,IRES,UROUND,JACD,RPAR,IPAR)
C         CJOLD=CJ
C         S = 100.D0
C         IF (IRES .LT. 0) GO TO 380
C         IF(IERJ .NE. 0)GO TO 380
C      ENDIF
CC
CC     Call the nonlinear Newton solver.
CC
C      TEMP1 = 2.0D0/(1.0D0 + CJ/CJOLD)
C      CALL DNSD(X,Y,YPRIME,NEQ,RES,PDUM,WT,RPAR,IPAR,DUMSVR,
C     *          DELTA,E,WM,IWM,CJ,DUMS,DUMR,DUME,EPCON,S,TEMP1,
C     *          TOLNEW,MULDEL,MAXIT,IRES,IDUM,IERNEW)
CC
C      IF (IERNEW .GT. 0 .AND. JCALC .NE. 0) THEN
CC
CC        The Newton iteration had a recoverable failure with an old
CC        iteration matrix.  Retry the step with a new iteration matrix.
CC
C         JCALC = -1
C         GO TO 300
C      ENDIF
CC
C      IF (IERNEW .NE. 0) GO TO 380
CC
CC     The Newton iteration has converged.  If nonnegativity of
CC     solution is required, set the solution nonnegative, if the
CC     perturbation to do it is small enough.  If the change is too
CC     large, then consider the corrector iteration to have failed.
CC
C375   IF(NONNEG .EQ. 0) GO TO 390
C      DO 377 I = 1,NEQ
C377      DELTA(I) = MIN(Y(I),0.0D0)
C      DELNRM = DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      IF(DELNRM .GT. EPCON) GO TO 380
C      DO 378 I = 1,NEQ
C378      E(I) = E(I) - DELTA(I)
C      GO TO 390
CC
CC
CC     Exits from nonlinear solver.
CC     No convergence with current iteration
CC     matrix, or singular iteration matrix.
CC     Compute IERNLS and IDID accordingly.
CC
C380   CONTINUE
C      IF (IRES .LE. -2 .OR. IERTYP .NE. 0) THEN
C         IERNLS = -1
C         IF (IRES .LE. -2) IDID = -11
C         IF (IERTYP .NE. 0) IDID = -15
C      ELSE
C         IERNLS = 1
C         IF (IRES .LT. 0) IDID = -10
C         IF (IERJ .NE. 0) IDID = -8
C      ENDIF
CC
C390   JCALC = 1
C      RETURN
CC
CC------END OF SUBROUTINE DNEDD------------------------------------------
C      END
C      SUBROUTINE DNSD(X,Y,YPRIME,NEQ,RES,PDUM,WT,RPAR,IPAR,
C     *   DUMSVR,DELTA,E,WM,IWM,CJ,DUMS,DUMR,DUME,EPCON,
C     *   S,CONFAC,TOLNEW,MULDEL,MAXIT,IRES,IDUM,IERNEW)
CC
CC***BEGIN PROLOGUE  DNSD
CC***REFER TO  DDASPK
CC***DATE WRITTEN   891219   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  950126   (YYMMDD)
CC***REVISION DATE  000711   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNSD solves a nonlinear system of
CC     algebraic equations of the form
CC     G(X,Y,YPRIME) = 0 for the unknown Y.
CC
CC     The method used is a modified Newton scheme.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     PDUM      -- Dummy argument.
CC     WT        -- Vector of weights for error criterion.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     DUMSVR    -- Dummy argument.
CC     DELTA     -- Work vector for DNSD of length NEQ.
CC     E         -- Error accumulation vector for DNSD of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information such as the matrix
CC                  of partial derivatives, permutation
CC                  vector, and various other information.
CC     CJ        -- Parameter always proportional to 1/H (step size).
CC     DUMS      -- Dummy argument.
CC     DUMR      -- Dummy argument.
CC     DUME      -- Dummy argument.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     S         -- Used for error convergence tests.
CC                  In the Newton iteration: S = RATE/(1 - RATE),
CC                  where RATE is the estimated rate of convergence
CC                  of the Newton iteration.
CC                  The calling routine passes the initial value
CC                  of S to the Newton iteration.
CC     CONFAC    -- A residual scale factor to improve convergence.
CC     TOLNEW    -- Tolerance on the norm of Newton correction in
CC                  alternative Newton convergence test.
CC     MULDEL    -- A flag indicating whether or not to multiply
CC                  DELTA by CONFAC.
CC                  0  ==> do not scale DELTA by CONFAC.
CC                  1  ==> scale DELTA by CONFAC.
CC     MAXIT     -- Maximum allowed number of Newton iterations.
CC     IRES      -- Error flag returned from RES.  See RES description
CC                  in DDASPK prologue.  If IRES = -1, then IERNEW
CC                  will be set to 1.
CC                  If IRES < -1, then IERNEW will be set to -1.
CC     IDUM      -- Dummy argument.
CC     IERNEW    -- Error flag for Newton iteration.
CC                   0  ==> Newton iteration converged.
CC                   1  ==> recoverable error inside Newton iteration.
CC                  -1  ==> unrecoverable error inside Newton iteration.
CC
CC     All arguments with "DUM" in their names are dummy arguments
CC     which are not used in this routine.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   DSLVD, DDWNRM, RES
CC
CC***END PROLOGUE  DNSD
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*),DELTA(*),E(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      EXTERNAL  RES
CC
C      PARAMETER (LNRE=12, LNNI=19)
CC
CC     Initialize Newton counter M and accumulation vector E. 
CC
C      M = 0
C      DO 100 I=1,NEQ
C100     E(I)=0.0D0
CC
CC     Corrector loop.
CC
C300   CONTINUE
C      IWM(LNNI) = IWM(LNNI) + 1
CC
CC     If necessary, multiply residual by convergence factor.
CC
C      IF (MULDEL .EQ. 1) THEN
C         DO 320 I = 1,NEQ
C320        DELTA(I) = DELTA(I) * CONFAC
C        ENDIF
CC
CC     Compute a new iterate (back-substitution).
CC     Store the correction in DELTA.
CC
C      CALL DSLVD(NEQ,DELTA,WM,IWM)
CC
CC     Update Y, E, and YPRIME.
CC
C      DO 340 I=1,NEQ
C         Y(I)=Y(I)-DELTA(I)
C         E(I)=E(I)-DELTA(I)
C340      YPRIME(I)=YPRIME(I)-CJ*DELTA(I)
CC
CC     Test for convergence of the iteration.
CC
C      DELNRM=DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      IF (M .EQ. 0) THEN
C        OLDNRM = DELNRM
C        IF (DELNRM .LE. TOLNEW) GO TO 370
C      ELSE
C        RATE = (DELNRM/OLDNRM)**(1.0D0/M)
C        IF (RATE .GT. 0.9D0) GO TO 380
C        S = RATE/(1.0D0 - RATE)
C      ENDIF
C      IF (S*DELNRM .LE. EPCON) GO TO 370
CC
CC     The corrector has not yet converged.
CC     Update M and test whether the
CC     maximum number of iterations have
CC     been tried.
CC
C      M=M+1
C      IF(M.GE.MAXIT) GO TO 380
CC
CC     Evaluate the residual,
CC     and go back to do another iteration.
CC
C      IWM(LNRE)=IWM(LNRE)+1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 380
C      GO TO 300
CC
CC     The iteration has converged.
CC
C370   RETURN
CC
CC     The iteration has not converged.  Set IERNEW appropriately.
CC
C380   CONTINUE
C      IF (IRES .LE. -2 ) THEN
C         IERNEW = -1
C      ELSE
C         IERNEW = 1
C      ENDIF
C      RETURN
CC
CC
CC------END OF SUBROUTINE DNSD-------------------------------------------
C      END
C      SUBROUTINE DMATD(NEQ,X,Y,YPRIME,DELTA,CJ,H,IER,EWT,E,
C     *                 WM,IWM,RES,IRES,UROUND,JACD,RPAR,IPAR)
CC
CC***BEGIN PROLOGUE  DMATD
CC***REFER TO  DDASPK
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940701   (YYMMDD) (new LIPVT)
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     This routine computes the iteration matrix
CC     J = dG/dY+CJ*dG/dYPRIME (where G(X,Y,YPRIME)=0).
CC     Here J is computed by:
CC       the user-supplied routine JACD if IWM(MTYPE) is 1 or 4, or
CC       by numerical difference quotients if IWM(MTYPE) is 2 or 5.
CC
CC     The parameters have the following meanings.
CC     X        = Independent variable.
CC     Y        = Array containing predicted values.
CC     YPRIME   = Array containing predicted derivatives.
CC     DELTA    = Residual evaluated at (X,Y,YPRIME).
CC                (Used only if IWM(MTYPE)=2 or 5).
CC     CJ       = Scalar parameter defining iteration matrix.
CC     H        = Current stepsize in integration.
CC     IER      = Variable which is .NE. 0 if iteration matrix
CC                is singular, and 0 otherwise.
CC     EWT      = Vector of error weights for computing norms.
CC     E        = Work space (temporary) of length NEQ.
CC     WM       = Real work space for matrices.  On output
CC                it contains the LU decomposition
CC                of the iteration matrix.
CC     IWM      = Integer work space containing
CC                matrix information.
CC     RES      = External user-supplied subroutine
CC                to evaluate the residual.  See RES description
CC                in DDASPK prologue.
CC     IRES     = Flag which is equal to zero if no illegal values
CC                in RES, and less than zero otherwise.  (If IRES
CC                is less than zero, the matrix was not completed).
CC                In this case (if IRES .LT. 0), then IER = 0.
CC     UROUND   = The unit roundoff error of the machine being used.
CC     JACD     = Name of the external user-supplied routine
CC                to evaluate the iteration matrix.  (This routine
CC                is only used if IWM(MTYPE) is 1 or 4)
CC                See JAC description for the case INFO(12) = 0
CC                in DDASPK prologue.
CC     RPAR,IPAR= Real and integer parameter arrays that
CC                are used for communication between the
CC                calling program and external user routines.
CC                They are not altered by DMATD.
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   JACD, RES, DGEFA, DGBFA
CC
CC***END PROLOGUE  DMATD
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),DELTA(*),EWT(*),E(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      EXTERNAL  RES, JACD
CC
C      PARAMETER (LML=1, LMU=2, LMTYPE=4, LNRE=12, LNPD=22, LLCIWP=30)
CC
C      LIPVT = IWM(LLCIWP)
C      IER = 0
C      MTYPE=IWM(LMTYPE)
C      GO TO (100,200,300,400,500),MTYPE
CC
CC
CC     Dense user-supplied matrix.
CC
C100   LENPD=IWM(LNPD)
C      DO 110 I=1,LENPD
C110      WM(I)=0.0D0
C      CALL JACD(X,Y,YPRIME,WM,CJ,RPAR,IPAR)
C      GO TO 230
CC
CC
CC     Dense finite-difference-generated matrix.
CC
C200   IRES=0
C      NROW=0
C      SQUR = SQRT(UROUND)
C      DO 210 I=1,NEQ
C         DEL=SQUR*MAX(ABS(Y(I)),ABS(H*YPRIME(I)),
C     *     ABS(1.D0/EWT(I)))
C         DEL=SIGN(DEL,H*YPRIME(I))
C         DEL=(Y(I)+DEL)-Y(I)
C         YSAVE=Y(I)
C         YPSAVE=YPRIME(I)
C         Y(I)=Y(I)+DEL
C         YPRIME(I)=YPRIME(I)+CJ*DEL
C         IWM(LNRE)=IWM(LNRE)+1
C         CALL RES(X,Y,YPRIME,CJ,E,IRES,RPAR,IPAR)
C         IF (IRES .LT. 0) RETURN
C         DELINV=1.0D0/DEL
C         DO 220 L=1,NEQ
C220        WM(NROW+L)=(E(L)-DELTA(L))*DELINV
C      NROW=NROW+NEQ
C      Y(I)=YSAVE
C      YPRIME(I)=YPSAVE
C210   CONTINUE
CC
CC
CC     Do dense-matrix LU decomposition on J.
CC
C230      CALL DGEFA(WM,NEQ,NEQ,IWM(LIPVT),IER)
C      RETURN
CC
CC
CC     Dummy section for IWM(MTYPE)=3.
CC
C300   RETURN
CC
CC
CC     Banded user-supplied matrix.
CC
C400   LENPD=IWM(LNPD)
C      DO 410 I=1,LENPD
C410      WM(I)=0.0D0
C      CALL JACD(X,Y,YPRIME,WM,CJ,RPAR,IPAR)
C      MEBAND=2*IWM(LML)+IWM(LMU)+1
C      GO TO 550
CC
CC
CC     Banded finite-difference-generated matrix.
CC
C500   MBAND=IWM(LML)+IWM(LMU)+1
C      MBA=MIN0(MBAND,NEQ)
C      MEBAND=MBAND+IWM(LML)
C      MEB1=MEBAND-1
C      MSAVE=(NEQ/MBAND)+1
C      ISAVE=IWM(LNPD)
C      IPSAVE=ISAVE+MSAVE
C      IRES=0
C      SQUR=SQRT(UROUND)
C      DO 540 J=1,MBA
C        DO 510 N=J,NEQ,MBAND
C          K= (N-J)/MBAND + 1
C          WM(ISAVE+K)=Y(N)
C          WM(IPSAVE+K)=YPRIME(N)
C          DEL=SQUR*MAX(ABS(Y(N)),ABS(H*YPRIME(N)),
C     *      ABS(1.D0/EWT(N)))
C          DEL=SIGN(DEL,H*YPRIME(N))
C          DEL=(Y(N)+DEL)-Y(N)
C          Y(N)=Y(N)+DEL
C510       YPRIME(N)=YPRIME(N)+CJ*DEL
C        IWM(LNRE)=IWM(LNRE)+1
C        CALL RES(X,Y,YPRIME,CJ,E,IRES,RPAR,IPAR)
C        IF (IRES .LT. 0) RETURN
C        DO 530 N=J,NEQ,MBAND
C          K= (N-J)/MBAND + 1
C          Y(N)=WM(ISAVE+K)
C          YPRIME(N)=WM(IPSAVE+K)
C          DEL=SQUR*MAX(ABS(Y(N)),ABS(H*YPRIME(N)),
C     *      ABS(1.D0/EWT(N)))
C          DEL=SIGN(DEL,H*YPRIME(N))
C          DEL=(Y(N)+DEL)-Y(N)
C          DELINV=1.0D0/DEL
C          I1=MAX0(1,(N-IWM(LMU)))
C          I2=MIN0(NEQ,(N+IWM(LML)))
C          II=N*MEB1-IWM(LML)
C          DO 520 I=I1,I2
C520         WM(II+I)=(E(I)-DELTA(I))*DELINV
C530     CONTINUE
C540   CONTINUE
CC
CC
CC     Do LU decomposition of banded J.
CC
C550   CALL DGBFA (WM,MEBAND,NEQ,IWM(LML),IWM(LMU),IWM(LIPVT),IER)
C      RETURN
CC
CC------END OF SUBROUTINE DMATD------------------------------------------
C      END
C      SUBROUTINE DSLVD(NEQ,DELTA,WM,IWM)
CC
CC***BEGIN PROLOGUE  DSLVD
CC***REFER TO  DDASPK
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940701   (YYMMDD) (new LIPVT)
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     This routine manages the solution of the linear
CC     system arising in the Newton iteration.
CC     Real matrix information and real temporary storage
CC     is stored in the array WM.
CC     Integer matrix information is stored in the array IWM.
CC     For a dense matrix, the LINPACK routine DGESL is called.
CC     For a banded matrix, the LINPACK routine DGBSL is called.
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   DGESL, DGBSL
CC
CC***END PROLOGUE  DSLVD
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION DELTA(*),WM(*),IWM(*)
CC
C      PARAMETER (LML=1, LMU=2, LMTYPE=4, LLCIWP=30)
CC
C      LIPVT = IWM(LLCIWP)
C      MTYPE=IWM(LMTYPE)
C      GO TO(100,100,300,400,400),MTYPE
CC
CC     Dense matrix.
CC
C100   CALL DGESL(WM,NEQ,NEQ,IWM(LIPVT),DELTA,0)
C      RETURN
CC
CC     Dummy section for MTYPE=3.
CC
C300   CONTINUE
C      RETURN
CC
CC     Banded matrix.
CC
C400   MEBAND=2*IWM(LML)+IWM(LMU)+1
C      CALL DGBSL(WM,MEBAND,NEQ,IWM(LML),
C     *  IWM(LMU),IWM(LIPVT),DELTA,0)
C      RETURN
CC
CC------END OF SUBROUTINE DSLVD------------------------------------------
C      END
C      SUBROUTINE DDASIK(X,Y,YPRIME,NEQ,ICOPT,ID,RES,JACK,PSOL,H,TSCALE,
C     *   WT,JSKIP,RPAR,IPAR,SAVR,DELTA,R,YIC,YPIC,PWK,WM,IWM,CJ,UROUND,
C     *   EPLI,SQRTN,RSQRTN,EPCON,RATEMX,STPTOL,JFLG,
C     *   ICNFLG,ICNSTR,IERNLS)
CC
CC***BEGIN PROLOGUE  DDASIK
CC***REFER TO  DDASPK
CC***DATE WRITTEN   941026   (YYMMDD)
CC***REVISION DATE  950808   (YYMMDD)
CC***REVISION DATE  951110   Removed unreachable block 390.
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC
CC     DDASIK solves a nonlinear system of algebraic equations of the
CC     form G(X,Y,YPRIME) = 0 for the unknown parts of Y and YPRIME in
CC     the initial conditions.
CC
CC     An initial value for Y and initial guess for YPRIME are input.
CC
CC     The method used is a Newton scheme with Krylov iteration and a
CC     linesearch algorithm.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector at x.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of equations to be integrated.
CC     ICOPT     -- Initial condition option chosen (1 or 2).
CC     ID        -- Array of dimension NEQ, which must be initialized
CC                  if ICOPT = 1.  See DDASIC.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     JACK     --  External user-supplied routine to update
CC                  the preconditioner.  (This is optional).
CC                  See JAC description for the case
CC                  INFO(12) = 1 in the DDASPK prologue.
CC     PSOL      -- External user-supplied routine to solve
CC                  a linear system using preconditioning.
CC                  (This is optional).  See explanation inside DDASPK.
CC     H         -- Scaling factor for this initial condition calc.
CC     TSCALE    -- Scale factor in T, used for stopping tests if nonzero.
CC     WT        -- Vector of weights for error criterion.
CC     JSKIP     -- input flag to signal if initial JAC call is to be
CC                  skipped.  1 => skip the call, 0 => do not skip call.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     SAVR      -- Work vector for DDASIK of length NEQ.
CC     DELTA     -- Work vector for DDASIK of length NEQ.
CC     R         -- Work vector for DDASIK of length NEQ.
CC     YIC,YPIC  -- Work vectors for DDASIK, each of length NEQ.
CC     PWK       -- Work vector for DDASIK of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information for linear system
CC                  solvers, and various other information.
CC     CJ        -- Matrix parameter = 1/H (ICOPT = 1) or 0 (ICOPT = 2).
CC     UROUND    -- Unit roundoff.  Not used here.
CC     EPLI      -- convergence test constant.
CC                  See DDASPK prologue for more details.
CC     SQRTN     -- Square root of NEQ.
CC     RSQRTN    -- reciprical of square root of NEQ.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     RATEMX    -- Maximum convergence rate for which Newton iteration
CC                  is considered converging.
CC     JFLG      -- Flag showing whether a Jacobian routine is supplied.
CC     ICNFLG    -- Integer scalar.  If nonzero, then constraint
CC                  violations in the proposed new approximate solution
CC                  will be checked for, and the maximum step length 
CC                  will be adjusted accordingly.
CC     ICNSTR    -- Integer array of length NEQ containing flags for
CC                  checking constraints.
CC     IERNLS    -- Error flag for nonlinear solver.
CC                   0   ==> nonlinear solver converged.
CC                   1,2 ==> recoverable error inside nonlinear solver.
CC                           1 => retry with current Y, YPRIME
CC                           2 => retry with original Y, YPRIME
CC                  -1   ==> unrecoverable error in nonlinear solver.
CC
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   RES, JACK, DNSIK, DCOPY
CC
CC***END PROLOGUE  DDASIK
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),ID(*),WT(*),ICNSTR(*)
C      DIMENSION SAVR(*),DELTA(*),R(*),YIC(*),YPIC(*),PWK(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      EXTERNAL RES, JACK, PSOL
CC
C      PARAMETER (LNRE=12, LNJE=13, LLOCWP=29, LLCIWP=30)
C      PARAMETER (LMXNIT=32, LMXNJ=33)
CC
CC
CC     Perform initializations.
CC
C      LWP = IWM(LLOCWP)
C      LIWP = IWM(LLCIWP)
C      MXNIT = IWM(LMXNIT)
C      MXNJ = IWM(LMXNJ)
C      IERNLS = 0
C      NJ = 0
C      EPLIN = EPLI*EPCON
CC
CC     Call RES to initialize DELTA.
CC
C      IRES = 0
C      IWM(LNRE) = IWM(LNRE) + 1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 370
CC
CC     Looping point for updating the preconditioner.
CC
C 300  CONTINUE
CC
CC     Initialize all error flags to zero.
CC
C      IERPJ = 0
C      IRES = 0
C      IERNEW = 0
CC
CC     If a Jacobian routine was supplied, call it.
CC
C      IF (JFLG .EQ. 1 .AND. JSKIP .EQ. 0) THEN
C        NJ = NJ + 1
C        IWM(LNJE)=IWM(LNJE)+1
C        CALL JACK (RES, IRES, NEQ, X, Y, YPRIME, WT, DELTA, R, H, CJ,
C     *     WM(LWP), IWM(LIWP), IERPJ, RPAR, IPAR)
C        IF (IRES .LT. 0 .OR. IERPJ .NE. 0) GO TO 370
C        ENDIF
C      JSKIP = 0
CC
CC     Call the nonlinear Newton solver for up to MXNIT iterations.
CC
C      CALL DNSIK(X,Y,YPRIME,NEQ,ICOPT,ID,RES,PSOL,WT,RPAR,IPAR,
C     *   SAVR,DELTA,R,YIC,YPIC,PWK,WM,IWM,CJ,TSCALE,SQRTN,RSQRTN,
C     *   EPLIN,EPCON,RATEMX,MXNIT,STPTOL,ICNFLG,ICNSTR,IERNEW)
CC
C      IF (IERNEW .EQ. 1 .AND. NJ .LT. MXNJ .AND. JFLG .EQ. 1) THEN
CC
CC       Up to MXNIT iterations were done, the convergence rate is < 1,
CC       a Jacobian routine is supplied, and the number of JACK calls
CC       is less than MXNJ.  
CC       Copy the residual SAVR to DELTA, call JACK, and try again.
CC
C        CALL DCOPY (NEQ,  SAVR, 1, DELTA, 1)
C        GO TO 300
C        ENDIF
CC
C      IF (IERNEW .NE. 0) GO TO 380
C      RETURN
CC
CC
CC     Unsuccessful exits from nonlinear solver.
CC     Set IERNLS accordingly.
CC
C 370  IERNLS = 2
C      IF (IRES .LE. -2) IERNLS = -1
C      RETURN
CC
C 380  IERNLS = MIN(IERNEW,2)
C      RETURN
CC
CC----------------------- END OF SUBROUTINE DDASIK-----------------------
C      END
C      SUBROUTINE DNSIK(X,Y,YPRIME,NEQ,ICOPT,ID,RES,PSOL,WT,RPAR,IPAR,
C     *   SAVR,DELTA,R,YIC,YPIC,PWK,WM,IWM,CJ,TSCALE,SQRTN,RSQRTN,EPLIN,
C     *   EPCON,RATEMX,MAXIT,STPTOL,ICNFLG,ICNSTR,IERNEW)
CC
CC***BEGIN PROLOGUE  DNSIK
CC***REFER TO  DDASPK
CC***DATE WRITTEN   940701   (YYMMDD)
CC***REVISION DATE  950714   (YYMMDD)
CC***REVISION DATE  000628   TSCALE argument added.
CC***REVISION DATE  000628   Added criterion for IERNEW = 1 return.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNSIK solves a nonlinear system of algebraic equations of the
CC     form G(X,Y,YPRIME) = 0 for the unknown parts of Y and YPRIME in
CC     the initial conditions.
CC
CC     The method used is a Newton scheme combined with a linesearch
CC     algorithm, using Krylov iterative linear system methods.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     ICOPT     -- Initial condition option chosen (1 or 2).
CC     ID        -- Array of dimension NEQ, which must be initialized
CC                  if ICOPT = 1.  See DDASIC.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     PSOL      -- External user-supplied routine to solve
CC                  a linear system using preconditioning. 
CC                  See explanation inside DDASPK.
CC     WT        -- Vector of weights for error criterion.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     SAVR      -- Work vector for DNSIK of length NEQ.
CC     DELTA     -- Residual vector on entry, and work vector of
CC                  length NEQ for DNSIK.
CC     R         -- Work vector for DNSIK of length NEQ.
CC     YIC,YPIC  -- Work vectors for DNSIK, each of length NEQ.
CC     PWK       -- Work vector for DNSIK of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information such as the matrix
CC                  of partial derivatives, permutation
CC                  vector, and various other information.
CC     CJ        -- Matrix parameter = 1/H (ICOPT = 1) or 0 (ICOPT = 2).
CC     TSCALE    -- Scale factor in T, used for stopping tests if nonzero.
CC     SQRTN     -- Square root of NEQ.
CC     RSQRTN    -- reciprical of square root of NEQ.
CC     EPLIN     -- Tolerance for linear system solver.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     RATEMX    -- Maximum convergence rate for which Newton iteration
CC                  is considered converging.
CC     MAXIT     -- Maximum allowed number of Newton iterations.
CC     STPTOL    -- Tolerance used in calculating the minimum lambda
CC                  value allowed.
CC     ICNFLG    -- Integer scalar.  If nonzero, then constraint
CC                  violations in the proposed new approximate solution
CC                  will be checked for, and the maximum step length
CC                  will be adjusted accordingly.
CC     ICNSTR    -- Integer array of length NEQ containing flags for
CC                  checking constraints.
CC     IERNEW    -- Error flag for Newton iteration.
CC                   0  ==> Newton iteration converged.
CC                   1  ==> failed to converge, but RATE .lt. 1, or the
CC                          residual norm was reduced by a factor of .1.
CC                   2  ==> failed to converge, RATE .gt. RATEMX.
CC                   3  ==> other recoverable error.
CC                  -1  ==> unrecoverable error inside Newton iteration.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   DFNRMK, DSLVK, DDWNRM, DLINSK, DCOPY
CC
CC***END PROLOGUE  DNSIK
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*),ID(*),DELTA(*),R(*),SAVR(*)
C      DIMENSION YIC(*),YPIC(*),PWK(*),WM(*),IWM(*), RPAR(*),IPAR(*)
C      DIMENSION ICNSTR(*)
C      EXTERNAL RES, PSOL
CC
C      PARAMETER (LNNI=19, LNPS=21, LLOCWP=29, LLCIWP=30)
C      PARAMETER (LLSOFF=35, LSTOL=14)
CC
CC
CC     Initializations.  M is the Newton iteration counter.
CC
C      LSOFF = IWM(LLSOFF)
C      M = 0
C      RATE = 1.0D0
C      LWP = IWM(LLOCWP)
C      LIWP = IWM(LLCIWP)
C      RLX = 0.4D0
CC
CC     Save residual in SAVR.
CC
C      CALL DCOPY (NEQ, DELTA, 1, SAVR, 1)
CC
CC     Compute norm of (P-inverse)*(residual).
CC
C      CALL DFNRMK (NEQ, Y, X, YPRIME, SAVR, R, CJ, TSCALE, WT,
C     *   SQRTN, RSQRTN, RES, IRES, PSOL, 1, IER, FNRM, EPLIN,
C     *   WM(LWP), IWM(LIWP), PWK, RPAR, IPAR)
C      IWM(LNPS) = IWM(LNPS) + 1
C      IF (IER .NE. 0) THEN
C        IERNEW = 3
C        RETURN
C      ENDIF
CC
CC     Return now if residual norm is .le. EPCON.
CC
C      IF (FNRM .LE. EPCON) RETURN
CC
CC     Newton iteration loop.
CC
C      FNRM0 = FNRM
C300   CONTINUE
C      IWM(LNNI) = IWM(LNNI) + 1
CC
CC     Compute a new step vector DELTA.
CC
C      CALL DSLVK (NEQ, Y, X, YPRIME, SAVR, DELTA, WT, WM, IWM,
C     *   RES, IRES, PSOL, IERSL, CJ, EPLIN, SQRTN, RSQRTN, RHOK,
C     *   RPAR, IPAR)
C      IF (IRES .NE. 0 .OR. IERSL .NE. 0) GO TO 390
CC
CC     Get norm of DELTA.  Return now if DELTA is zero.
CC
C      DELNRM = DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      IF (DELNRM .EQ. 0.0D0) RETURN
CC
CC     Call linesearch routine for global strategy and set RATE.
CC
C      OLDFNM = FNRM
CC
C      CALL DLINSK (NEQ, Y, X, YPRIME, SAVR, CJ, TSCALE, DELTA, DELNRM,
C     *   WT, SQRTN, RSQRTN, LSOFF, STPTOL, IRET, RES, IRES, PSOL,
C     *   WM, IWM, RHOK, FNRM, ICOPT, ID, WM(LWP), IWM(LIWP), R, EPLIN,
C     *   YIC, YPIC, PWK, ICNFLG, ICNSTR, RLX, RPAR, IPAR)
CC
C      RATE = FNRM/OLDFNM
CC
CC     Check for error condition from linesearch.
C      IF (IRET .NE. 0) GO TO 390
CC
CC     Test for convergence of the iteration, and return or loop.
CC
C      IF (FNRM .LE. EPCON) RETURN
CC
CC     The iteration has not yet converged.  Update M.
CC     Test whether the maximum number of iterations have been tried.
CC
C      M = M + 1
C      IF(M .GE. MAXIT) GO TO 380
CC
CC     Copy the residual SAVR to DELTA and loop for another iteration.
CC
C      CALL DCOPY (NEQ,  SAVR, 1, DELTA, 1)
C      GO TO 300
CC
CC     The maximum number of iterations was done.  Set IERNEW and return.
CC
C380   IF (RATE .LE. RATEMX .OR. FNRM .LE. 0.1D0*FNRM0) THEN
C         IERNEW = 1
C      ELSE
C         IERNEW = 2
C      ENDIF
C      RETURN
CC
C390   IF (IRES .LE. -2 .OR. IERSL .LT. 0) THEN
C         IERNEW = -1
C      ELSE
C         IERNEW = 3
C         IF (IRES .EQ. 0 .AND. IERSL .EQ. 1 .AND. M .GE. 2 
C     1       .AND. RATE .LT. 1.0D0) IERNEW = 1
C      ENDIF
C      RETURN
CC
CC
CC----------------------- END OF SUBROUTINE DNSIK------------------------
C      END
C      SUBROUTINE DLINSK (NEQ, Y, T, YPRIME, SAVR, CJ, TSCALE, P, PNRM,
C     *   WT, SQRTN, RSQRTN, LSOFF, STPTOL, IRET, RES, IRES, PSOL,
C     *   WM, IWM, RHOK, FNRM, ICOPT, ID, WP, IWP, R, EPLIN, YNEW, YPNEW,
C     *   PWK, ICNFLG, ICNSTR, RLX, RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DLINSK
CC***REFER TO  DNSIK
CC***DATE WRITTEN   940830   (YYMMDD)
CC***REVISION DATE  951006   (Arguments SQRTN, RSQRTN added.)
CC***REVISION DATE  960129   Moved line RL = ONE to top block.
CC***REVISION DATE  000628   TSCALE argument added.
CC***REVISION DATE  000628   RHOK*RHOK term removed in alpha test.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DLINSK uses a linesearch algorithm to calculate a new (Y,YPRIME)
CC     pair (YNEW,YPNEW) such that 
CC
CC     f(YNEW,YPNEW) .le. (1 - 2*ALPHA*RL)*f(Y,YPRIME)
CC
CC     where 0 < RL <= 1, and RHOK is the scaled preconditioned norm of
CC     the final residual vector in the Krylov iteration.  
CC     Here, f(y,y') is defined as
CC
CC      f(y,y') = (1/2)*norm( (P-inverse)*G(t,y,y') )**2 ,
CC
CC     where norm() is the weighted RMS vector norm, G is the DAE
CC     system residual function, and P is the preconditioner used
CC     in the Krylov iteration.
CC
CC     In addition to the parameters defined elsewhere, we have
CC
CC     SAVR    -- Work array of length NEQ, containing the residual
CC                vector G(t,y,y') on return.
CC     TSCALE  -- Scale factor in T, used for stopping tests if nonzero.
CC     P       -- Approximate Newton step used in backtracking.
CC     PNRM    -- Weighted RMS norm of P.
CC     LSOFF   -- Flag showing whether the linesearch algorithm is
CC                to be invoked.  0 means do the linesearch, 
CC                1 means turn off linesearch.
CC     STPTOL  -- Tolerance used in calculating the minimum lambda
CC                value allowed.
CC     ICNFLG  -- Integer scalar.  If nonzero, then constraint violations
CC                in the proposed new approximate solution will be
CC                checked for, and the maximum step length will be
CC                adjusted accordingly.
CC     ICNSTR  -- Integer array of length NEQ containing flags for
CC                checking constraints.
CC     RHOK    -- Weighted norm of preconditioned Krylov residual.
CC     RLX     -- Real scalar restricting update size in DCNSTR.
CC     YNEW    -- Array of length NEQ used to hold the new Y in
CC                performing the linesearch.
CC     YPNEW   -- Array of length NEQ used to hold the new YPRIME in
CC                performing the linesearch.
CC     PWK     -- Work vector of length NEQ for use in PSOL.
CC     Y       -- Array of length NEQ containing the new Y (i.e.,=YNEW).
CC     YPRIME  -- Array of length NEQ containing the new YPRIME 
CC                (i.e.,=YPNEW).
CC     FNRM    -- Real scalar containing SQRT(2*f(Y,YPRIME)) for the
CC                current (Y,YPRIME) on input and output.
CC     R       -- Work space length NEQ for residual vector.
CC     IRET    -- Return flag.
CC                IRET=0 means that a satisfactory (Y,YPRIME) was found.
CC                IRET=1 means that the routine failed to find a new
CC                       (Y,YPRIME) that was sufficiently distinct from
CC                       the current (Y,YPRIME) pair.
CC                IRET=2 means a failure in RES or PSOL.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   DFNRMK, DYYPNW, DCNSTR, DCOPY, XERRWD
CC
CC***END PROLOGUE  DLINSK
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      EXTERNAL  RES, PSOL
C      DIMENSION Y(*), YPRIME(*), P(*), WT(*), SAVR(*), R(*), ID(*)
C      DIMENSION WM(*), IWM(*), YNEW(*), YPNEW(*), PWK(*), ICNSTR(*)
C      DIMENSION WP(*), IWP(*), RPAR(*), IPAR(*)
C      CHARACTER MSG*80
CC
C      PARAMETER (LNRE=12, LNPS=21, LKPRIN=31)
CC
C      SAVE ALPHA, ONE, TWO
C      DATA ALPHA/1.0D-4/, ONE/1.0D0/, TWO/2.0D0/
CC
C      KPRIN=IWM(LKPRIN)
C      F1NRM = (FNRM*FNRM)/TWO
C      RATIO = ONE
CC
C      IF (KPRIN .GE. 2) THEN
C        MSG = '------ IN ROUTINE DLINSK-- PNRM = (R1)'
C        CALL XERRWD(MSG, 38, 921, 0, 0, 0, 0, 1, PNRM, 0.0D0)
C        ENDIF
C      TAU = PNRM
C      RL = ONE
CC-----------------------------------------------------------------------
CC Check for violations of the constraints, if any are imposed.
CC If any violations are found, the step vector P is rescaled, and the 
CC constraint check is repeated, until no violations are found.
CC-----------------------------------------------------------------------
C      IF (ICNFLG .NE. 0) THEN
C 10      CONTINUE
C         CALL DYYPNW (NEQ,Y,YPRIME,CJ,RL,P,ICOPT,ID,YNEW,YPNEW)
C         CALL DCNSTR (NEQ, Y, YNEW, ICNSTR, TAU, RLX, IRET, IVAR)
C         IF (IRET .EQ. 1) THEN
C            RATIO1 = TAU/PNRM
C            RATIO = RATIO*RATIO1
C            DO 20 I = 1,NEQ
C 20           P(I) = P(I)*RATIO1
C            PNRM = TAU
C            IF (KPRIN .GE. 2) THEN
C              MSG = '------ CONSTRAINT VIOL., PNRM = (R1), INDEX = (I1)'
C              CALL XERRWD(MSG, 50, 922, 0, 1, IVAR, 0, 1, PNRM, 0.0D0)
C              ENDIF
C            IF (PNRM .LE. STPTOL) THEN
C              IRET = 1
C              RETURN
C              ENDIF
C            GO TO 10
C            ENDIF
C         ENDIF
CC
C      SLPI = -TWO*F1NRM*RATIO
C      RLMIN = STPTOL/PNRM
C      IF (LSOFF .EQ. 0 .AND. KPRIN .GE. 2) THEN
C        MSG = '------ MIN. LAMBDA = (R1)'
C        CALL XERRWD(MSG, 25, 923, 0, 0, 0, 0, 1, RLMIN, 0.0D0)
C        ENDIF
CC-----------------------------------------------------------------------
CC Begin iteration to find RL value satisfying alpha-condition.
CC Update YNEW and YPNEW, then compute norm of new scaled residual and
CC perform alpha condition test.
CC-----------------------------------------------------------------------
C 100  CONTINUE
C      CALL DYYPNW (NEQ,Y,YPRIME,CJ,RL,P,ICOPT,ID,YNEW,YPNEW)
C      CALL DFNRMK (NEQ, YNEW, T, YPNEW, SAVR, R, CJ, TSCALE, WT,
C     *   SQRTN, RSQRTN, RES, IRES, PSOL, 0, IER, FNRMP, EPLIN,
C     *   WP, IWP, PWK, RPAR, IPAR)
C      IWM(LNRE) = IWM(LNRE) + 1
C      IF (IRES .GE. 0) IWM(LNPS) = IWM(LNPS) + 1
C      IF (IRES .NE. 0 .OR. IER .NE. 0) THEN
C        IRET = 2
C        RETURN
C        ENDIF
C      IF (LSOFF .EQ. 1) GO TO 150
CC
C      F1NRMP = FNRMP*FNRMP/TWO
C      IF (KPRIN .GE. 2) THEN
C        MSG = '------ LAMBDA = (R1)'
C        CALL XERRWD(MSG, 20, 924, 0, 0, 0, 0, 1, RL, 0.0D0)
C        MSG = '------ NORM(F1) = (R1),  NORM(F1NEW) = (R2)'
C        CALL XERRWD(MSG, 43, 925, 0, 0, 0, 0, 2, F1NRM, F1NRMP)
C        ENDIF
C      IF (F1NRMP .GT. F1NRM + ALPHA*SLPI*RL) GO TO 200
CC-----------------------------------------------------------------------
CC Alpha-condition is satisfied, or linesearch is turned off.
CC Copy YNEW,YPNEW to Y,YPRIME and return.
CC-----------------------------------------------------------------------
C 150  IRET = 0
C      CALL DCOPY(NEQ, YNEW, 1, Y, 1)
C      CALL DCOPY(NEQ, YPNEW, 1, YPRIME, 1)
C      FNRM = FNRMP
C      IF (KPRIN .GE. 1) THEN
C        MSG = '------ LEAVING ROUTINE DLINSK, FNRM = (R1)'
C        CALL XERRWD(MSG, 42, 926, 0, 0, 0, 0, 1, FNRM, 0.0D0)
C        ENDIF
C      RETURN
CC-----------------------------------------------------------------------
CC Alpha-condition not satisfied.  Perform backtrack to compute new RL
CC value.  If RL is less than RLMIN, i.e. no satisfactory YNEW,YPNEW can
CC be found sufficiently distinct from Y,YPRIME, then return IRET = 1.
CC-----------------------------------------------------------------------
C 200  CONTINUE
C      IF (RL .LT. RLMIN) THEN
C        IRET = 1
C        RETURN
C        ENDIF
CC
C      RL = RL/TWO
C      GO TO 100
CC
CC----------------------- END OF SUBROUTINE DLINSK ----------------------
C      END
C      SUBROUTINE DFNRMK (NEQ, Y, T, YPRIME, SAVR, R, CJ, TSCALE, WT,
C     *                   SQRTN, RSQRTN, RES, IRES, PSOL, IRIN, IER,
C     *                   FNORM, EPLIN, WP, IWP, PWK, RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DFNRMK
CC***REFER TO  DLINSK
CC***DATE WRITTEN   940830   (YYMMDD)
CC***REVISION DATE  951006   (SQRTN, RSQRTN, and scaling of WT added.)
CC***REVISION DATE  000628   TSCALE argument added.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DFNRMK calculates the scaled preconditioned norm of the nonlinear
CC     function used in the nonlinear iteration for obtaining consistent
CC     initial conditions.  Specifically, DFNRMK calculates the weighted
CC     root-mean-square norm of the vector (P-inverse)*G(T,Y,YPRIME),
CC     where P is the preconditioner matrix.
CC
CC     In addition to the parameters described in the calling program
CC     DLINSK, the parameters represent
CC
CC     TSCALE -- Scale factor in T, used for stopping tests if nonzero.
CC     IRIN   -- Flag showing whether the current residual vector is
CC               input in SAVR.  1 means it is, 0 means it is not.
CC     R      -- Array of length NEQ that contains
CC               (P-inverse)*G(T,Y,YPRIME) on return.
CC     FNORM  -- Scalar containing the weighted norm of R on return.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   RES, DCOPY, DSCAL, PSOL, DDWNRM
CC
CC***END PROLOGUE  DFNRMK
CC
CC
C      IMPLICIT DOUBLE PRECISION (A-H,O-Z)
C      EXTERNAL RES, PSOL
C      DIMENSION Y(*), YPRIME(*), WT(*), SAVR(*), R(*), PWK(*)
C      DIMENSION WP(*), IWP(*), RPAR(*), IPAR(*)
CC-----------------------------------------------------------------------
CC     Call RES routine if IRIN = 0.
CC-----------------------------------------------------------------------
C      IF (IRIN .EQ. 0) THEN
C        IRES = 0
C        CALL RES (T, Y, YPRIME, CJ, SAVR, IRES, RPAR, IPAR)
C        IF (IRES .LT. 0) RETURN
C        ENDIF
CC-----------------------------------------------------------------------
CC     Apply inverse of left preconditioner to vector R.
CC     First scale WT array by 1/sqrt(N), and undo scaling afterward.
CC-----------------------------------------------------------------------
C      CALL DCOPY(NEQ, SAVR, 1, R, 1)
C      CALL DSCAL (NEQ, RSQRTN, WT, 1)
C      IER = 0
C      CALL PSOL (NEQ, T, Y, YPRIME, SAVR, PWK, CJ, WT, WP, IWP,
C     *           R, EPLIN, IER, RPAR, IPAR)
C      CALL DSCAL (NEQ, SQRTN, WT, 1)
C      IF (IER .NE. 0) RETURN
CC-----------------------------------------------------------------------
CC     Calculate norm of R.
CC-----------------------------------------------------------------------
C      FNORM = DDWNRM (NEQ, R, WT, RPAR, IPAR)
C      IF (TSCALE .GT. 0.0D0) FNORM = FNORM*TSCALE*ABS(CJ)
CC
C      RETURN
CC----------------------- END OF SUBROUTINE DFNRMK ----------------------
C      END
C      SUBROUTINE DNEDK(X,Y,YPRIME,NEQ,RES,JACK,PSOL,
C     *   H,WT,JSTART,IDID,RPAR,IPAR,PHI,GAMMA,SAVR,DELTA,E,
C     *   WM,IWM,CJ,CJOLD,CJLAST,S,UROUND,EPLI,SQRTN,RSQRTN,
C     *   EPCON,JCALC,JFLG,KP1,NONNEG,NTYPE,IERNLS)
CC
CC***BEGIN PROLOGUE  DNEDK
CC***REFER TO  DDASPK
CC***DATE WRITTEN   891219   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940701   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNEDK solves a nonlinear system of
CC     algebraic equations of the form
CC     G(X,Y,YPRIME) = 0 for the unknown Y.
CC
CC     The method used is a matrix-free Newton scheme.
CC
CC     The parameters represent
CC     X         -- Independent variable.
CC     Y         -- Solution vector at x.
CC     YPRIME    -- Derivative of solution vector
CC                  after successful step.
CC     NEQ       -- Number of equations to be integrated.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     JACK     --  External user-supplied routine to update
CC                  the preconditioner.  (This is optional).
CC                  See JAC description for the case
CC                  INFO(12) = 1 in the DDASPK prologue.
CC     PSOL      -- External user-supplied routine to solve
CC                  a linear system using preconditioning. 
CC                  (This is optional).  See explanation inside DDASPK.
CC     H         -- Appropriate step size for this step.
CC     WT        -- Vector of weights for error criterion.
CC     JSTART    -- Indicates first call to this routine.
CC                  If JSTART = 0, then this is the first call,
CC                  otherwise it is not.
CC     IDID      -- Completion flag, output by DNEDK.
CC                  See IDID description in DDASPK prologue.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     PHI       -- Array of divided differences used by
CC                  DNEDK.  The length is NEQ*(K+1), where
CC                  K is the maximum order.
CC     GAMMA     -- Array used to predict Y and YPRIME.  The length
CC                  is K+1, where K is the maximum order.
CC     SAVR      -- Work vector for DNEDK of length NEQ.
CC     DELTA     -- Work vector for DNEDK of length NEQ.
CC     E         -- Error accumulation vector for DNEDK of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information for linear system
CC                  solvers, and various other information.
CC     CJ        -- Parameter always proportional to 1/H.
CC     CJOLD     -- Saves the value of CJ as of the last call to DITMD.
CC                  Accounts for changes in CJ needed to
CC                  decide whether to call DITMD.
CC     CJLAST    -- Previous value of CJ.
CC     S         -- A scalar determined by the approximate rate
CC                  of convergence of the Newton iteration and used
CC                  in the convergence test for the Newton iteration.
CC
CC                  If RATE is defined to be an estimate of the
CC                  rate of convergence of the Newton iteration,
CC                  then S = RATE/(1.D0-RATE).
CC
CC                  The closer RATE is to 0., the faster the Newton
CC                  iteration is converging; the closer RATE is to 1.,
CC                  the slower the Newton iteration is converging.
CC
CC                  On the first Newton iteration with an up-dated
CC                  preconditioner S = 100.D0, Thus the initial
CC                  RATE of convergence is approximately 1.
CC
CC                  S is preserved from call to call so that the rate
CC                  estimate from a previous step can be applied to
CC                  the current step.
CC     UROUND    -- Unit roundoff.  Not used here.
CC     EPLI      -- convergence test constant.
CC                  See DDASPK prologue for more details.
CC     SQRTN     -- Square root of NEQ.
CC     RSQRTN    -- reciprical of square root of NEQ.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     JCALC     -- Flag used to determine when to update
CC                  the Jacobian matrix.  In general:
CC
CC                  JCALC = -1 ==> Call the DITMD routine to update
CC                                 the Jacobian matrix.
CC                  JCALC =  0 ==> Jacobian matrix is up-to-date.
CC                  JCALC =  1 ==> Jacobian matrix is out-dated,
CC                                 but DITMD will not be called unless
CC                                 JCALC is set to -1.
CC     JFLG      -- Flag showing whether a Jacobian routine is supplied.
CC     KP1       -- The current order + 1;  updated across calls.
CC     NONNEG    -- Flag to determine nonnegativity constraints.
CC     NTYPE     -- Identification code for the DNEDK routine.
CC                   1 ==> modified Newton; iterative linear solver.
CC                   2 ==> modified Newton; user-supplied linear solver.
CC     IERNLS    -- Error flag for nonlinear solver.
CC                   0 ==> nonlinear solver converged.
CC                   1 ==> recoverable error inside non-linear solver.
CC                  -1 ==> unrecoverable error inside non-linear solver.
CC
CC     The following group of variables are passed as arguments to
CC     the Newton iteration solver.  They are explained in greater detail
CC     in DNSK:
CC        TOLNEW, MULDEL, MAXIT, IERNEW
CC
CC     IERTYP -- Flag which tells whether this subroutine is correct.
CC               0 ==> correct subroutine.
CC               1 ==> incorrect subroutine.
CC
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   RES, JACK, DDWNRM, DNSK
CC
CC***END PROLOGUE  DNEDK
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*)
C      DIMENSION PHI(NEQ,*),SAVR(*),DELTA(*),E(*)
C      DIMENSION WM(*),IWM(*)
C      DIMENSION GAMMA(*),RPAR(*),IPAR(*)
C      EXTERNAL  RES, JACK, PSOL
CC
C      PARAMETER (LNRE=12, LNJE=13, LLOCWP=29, LLCIWP=30)
CC
C      SAVE MULDEL, MAXIT, XRATE
C      DATA MULDEL/0/, MAXIT/4/, XRATE/0.25D0/
CC
CC     Verify that this is the correct subroutine.
CC
C      IERTYP = 0
C      IF (NTYPE .NE. 1) THEN
C         IERTYP = 1
C         GO TO 380
C         ENDIF
CC
CC     If this is the first step, perform initializations.
CC
C      IF (JSTART .EQ. 0) THEN
C         CJOLD = CJ
C         JCALC = -1
C         S = 100.D0
C         ENDIF
CC
CC     Perform all other initializations.
CC
C      IERNLS = 0
C      LWP = IWM(LLOCWP)
C      LIWP = IWM(LLCIWP)
CC
CC     Decide whether to update the preconditioner.
CC
C      IF (JFLG .NE. 0) THEN
C         TEMP1 = (1.0D0 - XRATE)/(1.0D0 + XRATE)
C         TEMP2 = 1.0D0/TEMP1
C         IF (CJ/CJOLD .LT. TEMP1 .OR. CJ/CJOLD .GT. TEMP2) JCALC = -1
C         IF (CJ .NE. CJLAST) S = 100.D0
C      ELSE
C         JCALC = 0
C         ENDIF
CC
CC     Looping point for updating preconditioner with current stepsize.
CC
C300   CONTINUE
CC
CC     Initialize all error flags to zero.
CC
C      IERPJ = 0
C      IRES = 0
C      IERSL = 0
C      IERNEW = 0
CC
CC     Predict the solution and derivative and compute the tolerance
CC     for the Newton iteration.
CC
C      DO 310 I=1,NEQ
C         Y(I)=PHI(I,1)
C310      YPRIME(I)=0.0D0
C      DO 330 J=2,KP1
C         DO 320 I=1,NEQ
C            Y(I)=Y(I)+PHI(I,J)
C320         YPRIME(I)=YPRIME(I)+GAMMA(J)*PHI(I,J)
C330   CONTINUE
C      EPLIN = EPLI*EPCON
C      TOLNEW = EPLIN
CC
CC     Call RES to initialize DELTA.
CC
C      IWM(LNRE)=IWM(LNRE)+1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 380
CC
CC
CC     If indicated, update the preconditioner.
CC     Set JCALC to 0 as an indicator that this has been done.
CC
C      IF(JCALC .EQ. -1)THEN
C         IWM(LNJE) = IWM(LNJE) + 1
C         JCALC=0
C         CALL JACK (RES, IRES, NEQ, X, Y, YPRIME, WT, DELTA, E, H, CJ,
C     *      WM(LWP), IWM(LIWP), IERPJ, RPAR, IPAR)
C         CJOLD=CJ
C         S = 100.D0
C         IF (IRES .LT. 0)  GO TO 380
C         IF (IERPJ .NE. 0) GO TO 380
C      ENDIF
CC
CC     Call the nonlinear Newton solver.
CC
C      CALL DNSK(X,Y,YPRIME,NEQ,RES,PSOL,WT,RPAR,IPAR,SAVR,
C     *   DELTA,E,WM,IWM,CJ,SQRTN,RSQRTN,EPLIN,EPCON,
C     *   S,TEMP1,TOLNEW,MULDEL,MAXIT,IRES,IERSL,IERNEW)
CC
C      IF (IERNEW .GT. 0 .AND. JCALC .NE. 0) THEN
CC
CC     The Newton iteration had a recoverable failure with an old
CC     preconditioner.  Retry the step with a new preconditioner.
CC
C         JCALC = -1
C         GO TO 300
C      ENDIF
CC
C      IF (IERNEW .NE. 0) GO TO 380
CC
CC     The Newton iteration has converged.  If nonnegativity of
CC     solution is required, set the solution nonnegative, if the
CC     perturbation to do it is small enough.  If the change is too
CC     large, then consider the corrector iteration to have failed.
CC
C      IF(NONNEG .EQ. 0) GO TO 390
C      DO 360 I = 1,NEQ
C 360    DELTA(I) = MIN(Y(I),0.0D0)
C      DELNRM = DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      IF(DELNRM .GT. EPCON) GO TO 380
C      DO 370 I = 1,NEQ
C 370    E(I) = E(I) - DELTA(I)
C      GO TO 390
CC
CC
CC     Exits from nonlinear solver.
CC     No convergence with current preconditioner.
CC     Compute IERNLS and IDID accordingly.
CC
C380   CONTINUE
C      IF (IRES .LE. -2 .OR. IERSL .LT. 0 .OR. IERTYP .NE. 0) THEN
C         IERNLS = -1
C         IF (IRES .LE. -2) IDID = -11
C         IF (IERSL .LT. 0) IDID = -13
C         IF (IERTYP .NE. 0) IDID = -15
C      ELSE
C         IERNLS =  1
C         IF (IRES .EQ. -1) IDID = -10
C         IF (IERPJ .NE. 0) IDID = -5
C         IF (IERSL .GT. 0) IDID = -14
C      ENDIF
CC
CC
C390   JCALC = 1
C      RETURN
CC
CC------END OF SUBROUTINE DNEDK------------------------------------------
C      END
C      SUBROUTINE DNSK(X,Y,YPRIME,NEQ,RES,PSOL,WT,RPAR,IPAR,
C     *   SAVR,DELTA,E,WM,IWM,CJ,SQRTN,RSQRTN,EPLIN,EPCON,
C     *   S,CONFAC,TOLNEW,MULDEL,MAXIT,IRES,IERSL,IERNEW)
CC
CC***BEGIN PROLOGUE  DNSK
CC***REFER TO  DDASPK
CC***DATE WRITTEN   891219   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  950126   (YYMMDD)
CC***REVISION DATE  000711   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC     DNSK solves a nonlinear system of
CC     algebraic equations of the form
CC     G(X,Y,YPRIME) = 0 for the unknown Y.
CC
CC     The method used is a modified Newton scheme.
CC
CC     The parameters represent
CC
CC     X         -- Independent variable.
CC     Y         -- Solution vector.
CC     YPRIME    -- Derivative of solution vector.
CC     NEQ       -- Number of unknowns.
CC     RES       -- External user-supplied subroutine
CC                  to evaluate the residual.  See RES description
CC                  in DDASPK prologue.
CC     PSOL      -- External user-supplied routine to solve
CC                  a linear system using preconditioning. 
CC                  See explanation inside DDASPK.
CC     WT        -- Vector of weights for error criterion.
CC     RPAR,IPAR -- Real and integer arrays used for communication
CC                  between the calling program and external user
CC                  routines.  They are not altered within DASPK.
CC     SAVR      -- Work vector for DNSK of length NEQ.
CC     DELTA     -- Work vector for DNSK of length NEQ.
CC     E         -- Error accumulation vector for DNSK of length NEQ.
CC     WM,IWM    -- Real and integer arrays storing
CC                  matrix information such as the matrix
CC                  of partial derivatives, permutation
CC                  vector, and various other information.
CC     CJ        -- Parameter always proportional to 1/H (step size).
CC     SQRTN     -- Square root of NEQ.
CC     RSQRTN    -- reciprical of square root of NEQ.
CC     EPLIN     -- Tolerance for linear system solver.
CC     EPCON     -- Tolerance to test for convergence of the Newton
CC                  iteration.
CC     S         -- Used for error convergence tests.
CC                  In the Newton iteration: S = RATE/(1.D0-RATE),
CC                  where RATE is the estimated rate of convergence
CC                  of the Newton iteration.
CC
CC                  The closer RATE is to 0., the faster the Newton
CC                  iteration is converging; the closer RATE is to 1.,
CC                  the slower the Newton iteration is converging.
CC
CC                  The calling routine sends the initial value
CC                  of S to the Newton iteration.
CC     CONFAC    -- A residual scale factor to improve convergence.
CC     TOLNEW    -- Tolerance on the norm of Newton correction in
CC                  alternative Newton convergence test.
CC     MULDEL    -- A flag indicating whether or not to multiply
CC                  DELTA by CONFAC.
CC                  0  ==> do not scale DELTA by CONFAC.
CC                  1  ==> scale DELTA by CONFAC.
CC     MAXIT     -- Maximum allowed number of Newton iterations.
CC     IRES      -- Error flag returned from RES.  See RES description
CC                  in DDASPK prologue.  If IRES = -1, then IERNEW
CC                  will be set to 1.
CC                  If IRES < -1, then IERNEW will be set to -1.
CC     IERSL     -- Error flag for linear system solver.
CC                  See IERSL description in subroutine DSLVK.
CC                  If IERSL = 1, then IERNEW will be set to 1.
CC                  If IERSL < 0, then IERNEW will be set to -1.
CC     IERNEW    -- Error flag for Newton iteration.
CC                   0  ==> Newton iteration converged.
CC                   1  ==> recoverable error inside Newton iteration.
CC                  -1  ==> unrecoverable error inside Newton iteration.
CC-----------------------------------------------------------------------
CC
CC***ROUTINES CALLED
CC   RES, DSLVK, DDWNRM
CC
CC***END PROLOGUE  DNSK
CC
CC
C      IMPLICIT DOUBLE PRECISION(A-H,O-Z)
C      DIMENSION Y(*),YPRIME(*),WT(*),DELTA(*),E(*),SAVR(*)
C      DIMENSION WM(*),IWM(*), RPAR(*),IPAR(*)
C      EXTERNAL  RES, PSOL
CC
C      PARAMETER (LNNI=19, LNRE=12)
CC
CC     Initialize Newton counter M and accumulation vector E.
CC
C      M = 0
C      DO 100 I=1,NEQ
C100     E(I) = 0.0D0
CC
CC     Corrector loop.
CC
C300   CONTINUE
C      IWM(LNNI) = IWM(LNNI) + 1
CC
CC     If necessary, multiply residual by convergence factor.
CC
C      IF (MULDEL .EQ. 1) THEN
C        DO 320 I = 1,NEQ
C320       DELTA(I) = DELTA(I) * CONFAC
C        ENDIF
CC
CC     Save residual in SAVR.
CC
C      DO 340 I = 1,NEQ
C340     SAVR(I) = DELTA(I)
CC
CC     Compute a new iterate.  Store the correction in DELTA.
CC
C      CALL DSLVK (NEQ, Y, X, YPRIME, SAVR, DELTA, WT, WM, IWM,
C     *   RES, IRES, PSOL, IERSL, CJ, EPLIN, SQRTN, RSQRTN, RHOK,
C     *   RPAR, IPAR)
C      IF (IRES .NE. 0 .OR. IERSL .NE. 0) GO TO 380
CC
CC     Update Y, E, and YPRIME.
CC
C      DO 360 I=1,NEQ
C         Y(I) = Y(I) - DELTA(I)
C         E(I) = E(I) - DELTA(I)
C360      YPRIME(I) = YPRIME(I) - CJ*DELTA(I)
CC
CC     Test for convergence of the iteration.
CC
C      DELNRM = DDWNRM(NEQ,DELTA,WT,RPAR,IPAR)
C      IF (M .EQ. 0) THEN
C        OLDNRM = DELNRM
C        IF (DELNRM .LE. TOLNEW) GO TO 370
C      ELSE
C        RATE = (DELNRM/OLDNRM)**(1.0D0/M)
C        IF (RATE .GT. 0.9D0) GO TO 380
C        S = RATE/(1.0D0 - RATE)
C      ENDIF
C      IF (S*DELNRM .LE. EPCON) GO TO 370
CC
CC     The corrector has not yet converged.  Update M and test whether
CC     the maximum number of iterations have been tried.
CC
C      M = M + 1
C      IF (M .GE. MAXIT) GO TO 380
CC
CC     Evaluate the residual, and go back to do another iteration.
CC
C      IWM(LNRE) = IWM(LNRE) + 1
C      CALL RES(X,Y,YPRIME,CJ,DELTA,IRES,RPAR,IPAR)
C      IF (IRES .LT. 0) GO TO 380
C      GO TO 300
CC
CC     The iteration has converged.
CC
C370    RETURN
CC
CC     The iteration has not converged.  Set IERNEW appropriately.
CC
C380   CONTINUE
C      IF (IRES .LE. -2 .OR. IERSL .LT. 0) THEN
C         IERNEW = -1
C      ELSE
C         IERNEW = 1
C      ENDIF
C      RETURN
CC
CC
CC------END OF SUBROUTINE DNSK-------------------------------------------
C      END
C      SUBROUTINE DSLVK (NEQ, Y, TN, YPRIME, SAVR, X, EWT, WM, IWM,
C     *   RES, IRES, PSOL, IERSL, CJ, EPLIN, SQRTN, RSQRTN, RHOK,
C     *   RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DSLVK
CC***REFER TO  DDASPK
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940928   Removed MNEWT and added RHOK in call list.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC DSLVK uses a restart algorithm and interfaces to DSPIGM for
CC the solution of the linear system arising from a Newton iteration.
CC
CC In addition to variables described elsewhere,
CC communication with DSLVK uses the following variables..
CC WM    = Real work space containing data for the algorithm
CC         (Krylov basis vectors, Hessenberg matrix, etc.).
CC IWM   = Integer work space containing data for the algorithm.
CC X     = The right-hand side vector on input, and the solution vector
CC         on output, of length NEQ.
CC IRES  = Error flag from RES.
CC IERSL = Output flag ..
CC         IERSL =  0 means no trouble occurred (or user RES routine
CC                    returned IRES < 0)
CC         IERSL =  1 means the iterative method failed to converge
CC                    (DSPIGM returned IFLAG > 0.)
CC         IERSL = -1 means there was a nonrecoverable error in the
CC                    iterative solver, and an error exit will occur.
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   DSCAL, DCOPY, DSPIGM
CC
CC***END PROLOGUE  DSLVK
CC
C      INTEGER NEQ, IWM, IRES, IERSL, IPAR
C      DOUBLE PRECISION Y, TN, YPRIME, SAVR, X, EWT, WM, CJ, EPLIN,
C     1   SQRTN, RSQRTN, RHOK, RPAR
C      DIMENSION Y(*), YPRIME(*), SAVR(*), X(*), EWT(*), 
C     1  WM(*), IWM(*), RPAR(*), IPAR(*)
CC
C      INTEGER IFLAG, IRST, NRSTS, NRMAX, LR, LDL, LHES, LGMR, LQ, LV,
C     1        LWK, LZ, MAXLP1, NPSL
C      INTEGER NLI, NPS, NCFL, NRE, MAXL, KMP, MITER
C      EXTERNAL  RES, PSOL
CC    
C      PARAMETER (LNRE=12, LNCFL=16, LNLI=20, LNPS=21) 
C      PARAMETER (LLOCWP=29, LLCIWP=30)
C      PARAMETER (LMITER=23, LMAXL=24, LKMP=25, LNRMAX=26)
CC
CC-----------------------------------------------------------------------
CC IRST is set to 1, to indicate restarting is in effect.
CC NRMAX is the maximum number of restarts.
CC-----------------------------------------------------------------------
C      DATA IRST/1/
CC
C      LIWP = IWM(LLCIWP)
C      NLI = IWM(LNLI)
C      NPS = IWM(LNPS)
C      NCFL = IWM(LNCFL)
C      NRE = IWM(LNRE)
C      LWP = IWM(LLOCWP)
C      MAXL = IWM(LMAXL) 
C      KMP = IWM(LKMP)
C      NRMAX = IWM(LNRMAX) 
C      MITER = IWM(LMITER)
C      IERSL = 0
C      IRES = 0
CC-----------------------------------------------------------------------
CC Use a restarting strategy to solve the linear system
CC P*X = -F.  Parse the work vector, and perform initializations.
CC Note that zero is the initial guess for X.
CC-----------------------------------------------------------------------
C      MAXLP1 = MAXL + 1
C      LV = 1
C      LR = LV + NEQ*MAXL
C      LHES = LR + NEQ + 1
C      LQ = LHES + MAXL*MAXLP1
C      LWK = LQ + 2*MAXL
C      LDL = LWK + MIN0(1,MAXL-KMP)*NEQ
C      LZ = LDL + NEQ
C      CALL DSCAL (NEQ, RSQRTN, EWT, 1)
C      CALL DCOPY (NEQ, X, 1, WM(LR), 1)
C      DO 110 I = 1,NEQ
C 110     X(I) = 0.D0
CC-----------------------------------------------------------------------
CC Top of loop for the restart algorithm.  Initial pass approximates
CC X and sets up a transformed system to perform subsequent restarts
CC to update X.  NRSTS is initialized to -1, because restarting
CC does not occur until after the first pass.
CC Update NRSTS; conditionally copy DL to R; call the DSPIGM
CC algorithm to solve A*Z = R;  updated counters;  update X with
CC the residual solution.
CC Note:  if convergence is not achieved after NRMAX restarts,
CC then the linear solver is considered to have failed.
CC-----------------------------------------------------------------------
C      NRSTS = -1
C 115  CONTINUE
C      NRSTS = NRSTS + 1
C      IF (NRSTS .GT. 0) CALL DCOPY (NEQ, WM(LDL), 1, WM(LR),1)
C      CALL DSPIGM (NEQ, TN, Y, YPRIME, SAVR, WM(LR), EWT, MAXL, MAXLP1,
C     1   KMP, EPLIN, CJ, RES, IRES, NRES, PSOL, NPSL, WM(LZ), WM(LV),
C     2   WM(LHES), WM(LQ), LGMR, WM(LWP), IWM(LIWP), WM(LWK),
C     3   WM(LDL), RHOK, IFLAG, IRST, NRSTS, RPAR, IPAR)
C      NLI = NLI + LGMR
C      NPS = NPS + NPSL
C      NRE = NRE + NRES
C      DO 120 I = 1,NEQ
C 120     X(I) = X(I) + WM(LZ+I-1) 
C      IF ((IFLAG .EQ. 1) .AND. (NRSTS .LT. NRMAX) .AND. (IRES .EQ. 0))
C     1   GO TO 115
CC-----------------------------------------------------------------------
CC The restart scheme is finished.  Test IRES and IFLAG to see if
CC convergence was not achieved, and set flags accordingly.
CC-----------------------------------------------------------------------
C      IF (IRES .LT. 0) THEN
C         NCFL = NCFL + 1
C      ELSE IF (IFLAG .NE. 0) THEN
C         NCFL = NCFL + 1
C         IF (IFLAG .GT. 0) IERSL = 1 
C         IF (IFLAG .LT. 0) IERSL = -1 
C      ENDIF
CC-----------------------------------------------------------------------
CC Update IWM with counters, rescale EWT, and return.
CC-----------------------------------------------------------------------
C      IWM(LNLI)  = NLI
C      IWM(LNPS)  = NPS
C      IWM(LNCFL) = NCFL
C      IWM(LNRE)  = NRE
C      CALL DSCAL (NEQ, SQRTN, EWT, 1)
C      RETURN
CC
CC------END OF SUBROUTINE DSLVK------------------------------------------
C      END
C      SUBROUTINE DSPIGM (NEQ, TN, Y, YPRIME, SAVR, R, WGHT, MAXL,
C     *   MAXLP1, KMP, EPLIN, CJ, RES, IRES, NRE, PSOL, NPSL, Z, V,
C     *   HES, Q, LGMR, WP, IWP, WK, DL, RHOK, IFLAG, IRST, NRSTS,
C     *   RPAR, IPAR)
CC
CC***BEGIN PROLOGUE  DSPIGM
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC***REVISION DATE  940927   Removed MNEWT and added RHOK in call list.
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC This routine solves the linear system A * Z = R using a scaled
CC preconditioned version of the generalized minimum residual method.
CC An initial guess of Z = 0 is assumed.
CC
CC      On entry
CC
CC          NEQ = Problem size, passed to PSOL.
CC
CC           TN = Current Value of T.
CC
CC            Y = Array Containing current dependent variable vector.
CC
CC       YPRIME = Array Containing current first derivative of Y.
CC
CC         SAVR = Array containing current value of G(T,Y,YPRIME).
CC
CC            R = The right hand side of the system A*Z = R.
CC                R is also used as work space when computing
CC                the final approximation and will therefore be
CC                destroyed.
CC                (R is the same as V(*,MAXL+1) in the call to DSPIGM.)
CC
CC         WGHT = The vector of length NEQ containing the nonzero
CC                elements of the diagonal scaling matrix.
CC
CC         MAXL = The maximum allowable order of the matrix H.
CC
CC       MAXLP1 = MAXL + 1, used for dynamic dimensioning of HES.
CC
CC          KMP = The number of previous vectors the new vector, VNEW,
CC                must be made orthogonal to.  (KMP .LE. MAXL.)
CC
CC        EPLIN = Tolerance on residuals R-A*Z in weighted rms norm.
CC
CC           CJ = Scalar proportional to current value of 
CC                1/(step size H).
CC
CC           WK = Real work array used by routine DATV1 and PSOL.
CC
CC           DL = Real work array used for calculation of the residual
CC                norm RHO when the method is incomplete (KMP.LT.MAXL)
CC                and/or when using restarting.
CC
CC           WP = Real work array used by preconditioner PSOL.
CC
CC          IWP = Integer work array used by preconditioner PSOL.
CC
CC         IRST = Method flag indicating if restarting is being
CC                performed.  IRST .GT. 0 means restarting is active,
CC                while IRST = 0 means restarting is not being used.
CC
CC        NRSTS = Counter for the number of restarts on the current
CC                call to DSPIGM.  If NRSTS .GT. 0, then the residual
CC                R is already scaled, and so scaling of R is not
CC                necessary.
CC
CC
CC      On Return
CC
CC         Z    = The final computed approximation to the solution
CC                of the system A*Z = R.
CC
CC         LGMR = The number of iterations performed and
CC                the current order of the upper Hessenberg
CC                matrix HES.
CC
CC         NRE  = The number of calls to RES (i.e. DATV1)
CC
CC         NPSL = The number of calls to PSOL.
CC
CC         V    = The neq by (LGMR+1) array containing the LGMR
CC                orthogonal vectors V(*,1) to V(*,LGMR).
CC
CC         HES  = The upper triangular factor of the QR decomposition
CC                of the (LGMR+1) by LGMR upper Hessenberg matrix whose
CC                entries are the scaled inner-products of A*V(*,I)
CC                and V(*,K).
CC
CC         Q    = Real array of length 2*MAXL containing the components
CC                of the givens rotations used in the QR decomposition
CC                of HES.  It is loaded in DHEQR and used in DHELS.
CC
CC         IRES = Error flag from RES.
CC
CC           DL = Scaled preconditioned residual, 
CC                (D-inverse)*(P-inverse)*(R-A*Z). Only loaded when
CC                performing restarts of the Krylov iteration.
CC
CC         RHOK = Weighted norm of final preconditioned residual.
CC
CC        IFLAG = Integer error flag..
CC                0 Means convergence in LGMR iterations, LGMR.LE.MAXL.
CC                1 Means the convergence test did not pass in MAXL
CC                  iterations, but the new residual norm (RHO) is
CC                  .LT. the old residual norm (RNRM), and so Z is
CC                  computed.
CC                2 Means the convergence test did not pass in MAXL
CC                  iterations, new residual norm (RHO) .GE. old residual
CC                  norm (RNRM), and the initial guess, Z = 0, is
CC                  returned.
CC                3 Means there was a recoverable error in PSOL
CC                  caused by the preconditioner being out of date.
CC               -1 Means there was an unrecoverable error in PSOL.
CC
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   PSOL, DNRM2, DSCAL, DATV1, DORTH, DHEQR, DCOPY, DHELS, DAXPY
CC
CC***END PROLOGUE  DSPIGM
CC
C      INTEGER NEQ,MAXL,MAXLP1,KMP,IRES,NRE,NPSL,LGMR,IWP,
C     1   IFLAG,IRST,NRSTS,IPAR
C      DOUBLE PRECISION TN,Y,YPRIME,SAVR,R,WGHT,EPLIN,CJ,Z,V,HES,Q,WP,WK,
C     1   DL,RHOK,RPAR
C      DIMENSION Y(*), YPRIME(*), SAVR(*), R(*), WGHT(*), Z(*),
C     1   V(NEQ,*), HES(MAXLP1,*), Q(*), WP(*), IWP(*), WK(*), DL(*),
C     2   RPAR(*), IPAR(*)
C      INTEGER I, IER, INFO, IP1, I2, J, K, LL, LLP1
C      DOUBLE PRECISION RNRM,C,DLNRM,PROD,RHO,S,SNORMW,DNRM2,TEM
C      EXTERNAL  RES, PSOL
CC
C      IER = 0
C      IFLAG = 0
C      LGMR = 0
C      NPSL = 0
C      NRE = 0
CC-----------------------------------------------------------------------
CC The initial guess for Z is 0.  The initial residual is therefore
CC the vector R.  Initialize Z to 0.
CC-----------------------------------------------------------------------
C      DO 10 I = 1,NEQ
C 10     Z(I) = 0.0D0
CC-----------------------------------------------------------------------
CC Apply inverse of left preconditioner to vector R if NRSTS .EQ. 0.
CC Form V(*,1), the scaled preconditioned right hand side.
CC-----------------------------------------------------------------------
C      IF (NRSTS .EQ. 0) THEN
C         CALL PSOL (NEQ, TN, Y, YPRIME, SAVR, WK, CJ, WGHT, WP, IWP,
C     1      R, EPLIN, IER, RPAR, IPAR)
C         NPSL = 1
C         IF (IER .NE. 0) GO TO 300
C         DO 30 I = 1,NEQ
C 30         V(I,1) = R(I)*WGHT(I)
C      ELSE
C         DO 35 I = 1,NEQ
C 35         V(I,1) = R(I)
C      ENDIF
CC-----------------------------------------------------------------------
CC Calculate norm of scaled vector V(*,1) and normalize it
CC If, however, the norm of V(*,1) (i.e. the norm of the preconditioned
CC residual) is .le. EPLIN, then return with Z=0.
CC-----------------------------------------------------------------------
C      RNRM = DNRM2 (NEQ, V, 1)
C      IF (RNRM .LE. EPLIN) THEN
C        RHOK = RNRM
C        RETURN
C        ENDIF
C      TEM = 1.0D0/RNRM
C      CALL DSCAL (NEQ, TEM, V(1,1), 1)
CC-----------------------------------------------------------------------
CC Zero out the HES array.
CC-----------------------------------------------------------------------
C      DO 65 J = 1,MAXL
C        DO 60 I = 1,MAXLP1
C 60       HES(I,J) = 0.0D0
C 65     CONTINUE
CC-----------------------------------------------------------------------
CC Main loop to compute the vectors V(*,2) to V(*,MAXL).
CC The running product PROD is needed for the convergence test.
CC-----------------------------------------------------------------------
C      PROD = 1.0D0
C      DO 90 LL = 1,MAXL
C        LGMR = LL
CC-----------------------------------------------------------------------
CC Call routine DATV1 to compute VNEW = ABAR*V(LL), where ABAR is
CC the matrix A with scaling and inverse preconditioner factors applied.
CC Call routine DORTH to orthogonalize the new vector VNEW = V(*,LL+1).
CC call routine DHEQR to update the factors of HES.
CC-----------------------------------------------------------------------
C        CALL DATV1 (NEQ, Y, TN, YPRIME, SAVR, V(1,LL), WGHT, Z,
C     1     RES, IRES, PSOL, V(1,LL+1), WK, WP, IWP, CJ, EPLIN,
C     1     IER, NRE, NPSL, RPAR, IPAR)
C        IF (IRES .LT. 0) RETURN
C        IF (IER .NE. 0) GO TO 300
C        CALL DORTH (V(1,LL+1), V, HES, NEQ, LL, MAXLP1, KMP, SNORMW)
C        HES(LL+1,LL) = SNORMW
C        CALL DHEQR (HES, MAXLP1, LL, Q, INFO, LL)
C        IF (INFO .EQ. LL) GO TO 120
CC-----------------------------------------------------------------------
CC Update RHO, the estimate of the norm of the residual R - A*ZL.
CC If KMP .LT. MAXL, then the vectors V(*,1),...,V(*,LL+1) are not
CC necessarily orthogonal for LL .GT. KMP.  The vector DL must then
CC be computed, and its norm used in the calculation of RHO.
CC-----------------------------------------------------------------------
C        PROD = PROD*Q(2*LL)
C        RHO = ABS(PROD*RNRM)
C        IF ((LL.GT.KMP) .AND. (KMP.LT.MAXL)) THEN
C          IF (LL .EQ. KMP+1) THEN
C            CALL DCOPY (NEQ, V(1,1), 1, DL, 1)
C            DO 75 I = 1,KMP
C              IP1 = I + 1
C              I2 = I*2
C              S = Q(I2)
C              C = Q(I2-1)
C              DO 70 K = 1,NEQ
C 70             DL(K) = S*DL(K) + C*V(K,IP1)
C 75           CONTINUE
C            ENDIF
C          S = Q(2*LL)
C          C = Q(2*LL-1)/SNORMW
C          LLP1 = LL + 1
C          DO 80 K = 1,NEQ
C 80         DL(K) = S*DL(K) + C*V(K,LLP1)
C          DLNRM = DNRM2 (NEQ, DL, 1)
C          RHO = RHO*DLNRM
C          ENDIF
CC-----------------------------------------------------------------------
CC Test for convergence.  If passed, compute approximation ZL.
CC If failed and LL .LT. MAXL, then continue iterating.
CC-----------------------------------------------------------------------
C        IF (RHO .LE. EPLIN) GO TO 200
C        IF (LL .EQ. MAXL) GO TO 100
CC-----------------------------------------------------------------------
CC Rescale so that the norm of V(1,LL+1) is one.
CC-----------------------------------------------------------------------
C        TEM = 1.0D0/SNORMW
C        CALL DSCAL (NEQ, TEM, V(1,LL+1), 1)
C 90     CONTINUE
C 100  CONTINUE
C      IF (RHO .LT. RNRM) GO TO 150
C 120  CONTINUE
C      IFLAG = 2
C      DO 130 I = 1,NEQ
C 130     Z(I) = 0.D0
C      RETURN
C 150  IFLAG = 1
CC-----------------------------------------------------------------------
CC The tolerance was not met, but the residual norm was reduced.
CC If performing restarting (IRST .gt. 0) calculate the residual vector
CC RL and store it in the DL array.  If the incomplete version is 
CC being used (KMP .lt. MAXL) then DL has already been calculated.
CC-----------------------------------------------------------------------
C      IF (IRST .GT. 0) THEN
C         IF (KMP .EQ. MAXL) THEN
CC
CC           Calculate DL from the V(I)'s.
CC
C            CALL DCOPY (NEQ, V(1,1), 1, DL, 1)
C            MAXLM1 = MAXL - 1
C            DO 175 I = 1,MAXLM1
C               IP1 = I + 1
C               I2 = I*2
C               S = Q(I2)
C               C = Q(I2-1)
C               DO 170 K = 1,NEQ
C 170              DL(K) = S*DL(K) + C*V(K,IP1)
C 175        CONTINUE
C            S = Q(2*MAXL)
C            C = Q(2*MAXL-1)/SNORMW
C            DO 180 K = 1,NEQ
C 180           DL(K) = S*DL(K) + C*V(K,MAXLP1)
C         ENDIF
CC
CC        Scale DL by RNRM*PROD to obtain the residual RL.
CC
C         TEM = RNRM*PROD
C         CALL DSCAL(NEQ, TEM, DL, 1)
C      ENDIF
CC-----------------------------------------------------------------------
CC Compute the approximation ZL to the solution.
CC Since the vector Z was used as work space, and the initial guess
CC of the Newton correction is zero, Z must be reset to zero.
CC-----------------------------------------------------------------------
C 200  CONTINUE
C      LL = LGMR
C      LLP1 = LL + 1
C      DO 210 K = 1,LLP1
C 210    R(K) = 0.0D0
C      R(1) = RNRM
C      CALL DHELS (HES, MAXLP1, LL, Q, R)
C      DO 220 K = 1,NEQ
C 220    Z(K) = 0.0D0
C      DO 230 I = 1,LL
C        CALL DAXPY (NEQ, R(I), V(1,I), 1, Z, 1)
C 230    CONTINUE
C      DO 240 I = 1,NEQ
C 240    Z(I) = Z(I)/WGHT(I)
CC Load RHO into RHOK.
C      RHOK = RHO
C      RETURN
CC-----------------------------------------------------------------------
CC This block handles error returns forced by routine PSOL.
CC-----------------------------------------------------------------------
C 300  CONTINUE
C      IF (IER .LT. 0) IFLAG = -1
C      IF (IER .GT. 0) IFLAG = 3
CC
C      RETURN
CC
CC------END OF SUBROUTINE DSPIGM-----------------------------------------
C      END
      SUBROUTINE DATV1 (NEQ, Y, TN, YPRIME, SAVR, V, WGHT, YPTEM, RES,
     *   IRES, PSOL, Z, VTEM, WP, IWP, CJ, EPLIN, IER, NRE, NPSL,
     *   RPAR,IPAR)
C
C***BEGIN PROLOGUE  DATV1
C***DATE WRITTEN   890101   (YYMMDD)
C***REVISION DATE  900926   (YYMMDD)
C
C
C-----------------------------------------------------------------------
C***DESCRIPTION
C
C This routine computes the product
C
C   Z = (D-inverse)*(P-inverse)*(dF/dY)*(D*V),
C
C where F(Y) = G(T, Y, CJ*(Y-A)), CJ is a scalar proportional to 1/H,
C and A involves the past history of Y.  The quantity CJ*(Y-A) is
C an approximation to the first derivative of Y and is stored
C in the array YPRIME.  Note that dF/dY = dG/dY + CJ*dG/dYPRIME.
C
C D is a diagonal scaling matrix, and P is the left preconditioning
C matrix.  V is assumed to have L2 norm equal to 1.
C The product is stored in Z and is computed by means of a
C difference quotient, a call to RES, and one call to PSOL.
C
C      On entry
C
C          NEQ = Problem size, passed to RES and PSOL.
C
C            Y = Array containing current dependent variable vector.
C
C       YPRIME = Array containing current first derivative of y.
C
C         SAVR = Array containing current value of G(T,Y,YPRIME).
C
C            V = Real array of length NEQ (can be the same array as Z).
C
C         WGHT = Array of length NEQ containing scale factors.
C                1/WGHT(I) are the diagonal elements of the matrix D.
C
C        YPTEM = Work array of length NEQ.
C
C         VTEM = Work array of length NEQ used to store the
C                unscaled version of V.
C
C         WP = Real work array used by preconditioner PSOL.
C
C         IWP = Integer work array used by preconditioner PSOL.
C
C           CJ = Scalar proportional to current value of 
C                1/(step size H).
C
C
C      On return
C
C            Z = Array of length NEQ containing desired scaled
C                matrix-vector product.
C
C         IRES = Error flag from RES.
C
C          IER = Error flag from PSOL.
C
C         NRE  = The number of calls to RES.
C
C         NPSL = The number of calls to PSOL.
C
C-----------------------------------------------------------------------
C***ROUTINES CALLED
C   RES, PSOL
C
C***END PROLOGUE  DATV1
C
      INTEGER NEQ, IRES, IWP, IER, NRE, NPSL, IPAR
      DOUBLE PRECISION Y, TN, YPRIME, SAVR, V, WGHT, YPTEM, Z, VTEM,
     1   WP, CJ, RPAR
      DIMENSION Y(*), YPRIME(*), SAVR(*), V(*), WGHT(*), YPTEM(*),
     1   Z(*), VTEM(*), WP(*), IWP(*), RPAR(*), IPAR(*)
      INTEGER I
      DOUBLE PRECISION EPLIN
      EXTERNAL  RES, PSOL
C
      IRES = 0
C-----------------------------------------------------------------------
C Set VTEM = D * V.
C-----------------------------------------------------------------------
      DO 10 I = 1,NEQ
 10     VTEM(I) = V(I)/WGHT(I)
      IER = 0
C-----------------------------------------------------------------------
C Store Y in Z and increment Z by VTEM.
C Store YPRIME in YPTEM and increment YPTEM by VTEM*CJ.
C-----------------------------------------------------------------------
      DO 20 I = 1,NEQ
        YPTEM(I) = YPRIME(I) + VTEM(I)*CJ
 20     Z(I) = Y(I) + VTEM(I)
C-----------------------------------------------------------------------
C Call RES with incremented Y, YPRIME arguments
C stored in Z, YPTEM.  VTEM is overwritten with new residual.
C-----------------------------------------------------------------------
      CONTINUE
      CALL RES(TN,Z,YPTEM,CJ,VTEM,IRES,RPAR,IPAR)
      NRE = NRE + 1
      IF (IRES .LT. 0) RETURN
C-----------------------------------------------------------------------
C Set Z = (dF/dY) * VBAR using difference quotient.
C (VBAR is old value of VTEM before calling RES)
C-----------------------------------------------------------------------
      DO 70 I = 1,NEQ
 70     Z(I) = VTEM(I) - SAVR(I)
C-----------------------------------------------------------------------
C Apply inverse of left preconditioner to Z.
C-----------------------------------------------------------------------
      CALL PSOL (NEQ, TN, Y, YPRIME, SAVR, YPTEM, CJ, WGHT, WP, IWP,
     1   Z, EPLIN, IER, RPAR, IPAR)
      NPSL = NPSL + 1
      IF (IER .NE. 0) RETURN
C-----------------------------------------------------------------------
C Apply D-inverse to Z and return.
C-----------------------------------------------------------------------
      DO 90 I = 1,NEQ
 90     Z(I) = Z(I)*WGHT(I)
      RETURN
C
C------END OF SUBROUTINE DATV1-------------------------------------------
      END
C      SUBROUTINE DORTH (VNEW, V, HES, N, LL, LDHES, KMP, SNORMW)
CC
CC***BEGIN PROLOGUE  DORTH
CC***DATE WRITTEN   890101   (YYMMDD)
CC***REVISION DATE  900926   (YYMMDD)
CC
CC
CC-----------------------------------------------------------------------
CC***DESCRIPTION
CC
CC This routine orthogonalizes the vector VNEW against the previous
CC KMP vectors in the V array.  It uses a modified Gram-Schmidt
CC orthogonalization procedure with conditional reorthogonalization.
CC
CC      On entry
CC
CC         VNEW = The vector of length N containing a scaled product
CC                OF The Jacobian and the vector V(*,LL).
CC
CC         V    = The N x LL array containing the previous LL
CC                orthogonal vectors V(*,1) to V(*,LL).
CC
CC         HES  = An LL x LL upper Hessenberg matrix containing,
CC                in HES(I,K), K.LT.LL, scaled inner products of
CC                A*V(*,K) and V(*,I).
CC
CC        LDHES = The leading dimension of the HES array.
CC
CC         N    = The order of the matrix A, and the length of VNEW.
CC
CC         LL   = The current order of the matrix HES.
CC
CC          KMP = The number of previous vectors the new vector VNEW
CC                must be made orthogonal to (KMP .LE. MAXL).
CC
CC
CC      On return
CC
CC         VNEW = The new vector orthogonal to V(*,I0),
CC                where I0 = MAX(1, LL-KMP+1).
CC
CC         HES  = Upper Hessenberg matrix with column LL filled in with
CC                scaled inner products of A*V(*,LL) and V(*,I).
CC
CC       SNORMW = L-2 norm of VNEW.
CC
CC-----------------------------------------------------------------------
CC***ROUTINES CALLED
CC   DDOT, DNRM2, DAXPY 
CC
CC***END PROLOGUE  DORTH
CC
C      INTEGER N, LL, LDHES, KMP
C      DOUBLE PRECISION VNEW, V, HES, SNORMW
C      DIMENSION VNEW(*), V(N,*), HES(LDHES,*)
C      INTEGER I, I0
C      DOUBLE PRECISION ARG, DDOT, DNRM2, SUMDSQ, TEM, VNRM
CC
CC-----------------------------------------------------------------------
CC Get norm of unaltered VNEW for later use.
CC-----------------------------------------------------------------------
C      VNRM = DNRM2 (N, VNEW, 1)
CC-----------------------------------------------------------------------
CC Do Modified Gram-Schmidt on VNEW = A*V(LL).
CC Scaled inner products give new column of HES.
CC Projections of earlier vectors are subtracted from VNEW.
CC-----------------------------------------------------------------------
C      I0 = MAX0(1,LL-KMP+1)
C      DO 10 I = I0,LL
C        HES(I,LL) = DDOT (N, V(1,I), 1, VNEW, 1)
C        TEM = -HES(I,LL)
C        CALL DAXPY (N, TEM, V(1,I), 1, VNEW, 1)
C 10     CONTINUE
CC-----------------------------------------------------------------------
CC Compute SNORMW = norm of VNEW.
CC If VNEW is small compared to its input value (in norm), then
CC Reorthogonalize VNEW to V(*,1) through V(*,LL).
CC Correct if relative correction exceeds 1000*(unit roundoff).
CC Finally, correct SNORMW using the dot products involved.
CC-----------------------------------------------------------------------
C      SNORMW = DNRM2 (N, VNEW, 1)
C      IF (VNRM + 0.001D0*SNORMW .NE. VNRM) RETURN
C      SUMDSQ = 0.0D0
C      DO 30 I = I0,LL
C        TEM = -DDOT (N, V(1,I), 1, VNEW, 1)
C        IF (HES(I,LL) + 0.001D0*TEM .EQ. HES(I,LL)) GO TO 30
C        HES(I,LL) = HES(I,LL) - TEM
C        CALL DAXPY (N, TEM, V(1,I), 1, VNEW, 1)
C        SUMDSQ = SUMDSQ + TEM**2
C 30     CONTINUE
C      IF (SUMDSQ .EQ. 0.0D0) RETURN
C      ARG = MAX(0.0D0,SNORMW**2 - SUMDSQ)
C      SNORMW = SQRT(ARG)
C      RETURN
CC
CC------END OF SUBROUTINE DORTH------------------------------------------
C      END
CC      SUBROUTINE DHEQR (A, LDA, N, Q, INFO, IJOB)
CCC
CCC***BEGIN PROLOGUE  DHEQR
CCC***DATE WRITTEN   890101   (YYMMDD)
CCC***REVISION DATE  900926   (YYMMDD)
CCC
CCC-----------------------------------------------------------------------
CCC***DESCRIPTION
CCC
CCC     This routine performs a QR decomposition of an upper
CCC     Hessenberg matrix A.  There are two options available:
CCC
CCC          (1)  performing a fresh decomposition
CCC          (2)  updating the QR factors by adding a row and A
CCC               column to the matrix A.
CCC
CCC     DHEQR decomposes an upper Hessenberg matrix by using Givens
CCC     rotations.
CCC
CCC     On entry
CCC
CCC        A       DOUBLE PRECISION(LDA, N)
CCC                The matrix to be decomposed.
CCC
CCC        LDA     INTEGER
CCC                The leading dimension of the array A.
CCC
CCC        N       INTEGER
CCC                A is an (N+1) by N Hessenberg matrix.
CCC
CCC        IJOB    INTEGER
CCC                = 1     Means that a fresh decomposition of the
CCC                        matrix A is desired.
CCC                .GE. 2  Means that the current decomposition of A
CCC                        will be updated by the addition of a row
CCC                        and a column.
CCC     On return
CCC
CCC        A       The upper triangular matrix R.
CCC                The factorization can be written Q*A = R, where
CCC                Q is a product of Givens rotations and R is upper
CCC                triangular.
CCC
CCC        Q       DOUBLE PRECISION(2*N)
CCC                The factors C and S of each Givens rotation used
CCC                in decomposing A.
CCC
CCC        INFO    INTEGER
CCC                = 0  normal value.
CCC                = K  If  A(K,K) .EQ. 0.0.  This is not an error
CCC                     condition for this subroutine, but it does
CCC                     indicate that DHELS will divide by zero
CCC                     if called.
CCC
CCC     Modification of LINPACK.
CCC     Peter Brown, Lawrence Livermore Natl. Lab.
CCC
CCC-----------------------------------------------------------------------
CCC***ROUTINES CALLED (NONE)
CCC
CCC***END PROLOGUE  DHEQR
CCC
CC      INTEGER LDA, N, INFO, IJOB
CC      DOUBLE PRECISION A(LDA,*), Q(*)
CC      INTEGER I, IQ, J, K, KM1, KP1, NM1
CC      DOUBLE PRECISION C, S, T, T1, T2
CCC
CC      IF (IJOB .GT. 1) GO TO 70
CCC-----------------------------------------------------------------------
CCC A new factorization is desired.
CCC-----------------------------------------------------------------------
CCC
CCC     QR decomposition without pivoting.
CCC
CC      INFO = 0
CC      DO 60 K = 1, N
CC         KM1 = K - 1
CC         KP1 = K + 1
CCC
CCC           Compute Kth column of R.
CCC           First, multiply the Kth column of A by the previous
CCC           K-1 Givens rotations.
CCC
CC            IF (KM1 .LT. 1) GO TO 20
CC            DO 10 J = 1, KM1
CC              I = 2*(J-1) + 1
CC              T1 = A(J,K)
CC              T2 = A(J+1,K)
CC              C = Q(I)
CC              S = Q(I+1)
CC              A(J,K) = C*T1 - S*T2
CC              A(J+1,K) = S*T1 + C*T2
CC   10         CONTINUE
CCC
CCC           Compute Givens components C and S.
CCC
CC   20       CONTINUE
CC            IQ = 2*KM1 + 1
CC            T1 = A(K,K)
CC            T2 = A(KP1,K)
CC            IF (T2 .NE. 0.0D0) GO TO 30
CC              C = 1.0D0
CC              S = 0.0D0
CC              GO TO 50
CC   30       CONTINUE
CC            IF (ABS(T2) .LT. ABS(T1)) GO TO 40
CC              T = T1/T2
CC              S = -1.0D0/SQRT(1.0D0+T*T)
CC              C = -S*T
CC              GO TO 50
CC   40       CONTINUE
CC              T = T2/T1
CC              C = 1.0D0/SQRT(1.0D0+T*T)
CC              S = -C*T
CC   50       CONTINUE
CC            Q(IQ) = C
CC            Q(IQ+1) = S
CC            A(K,K) = C*T1 - S*T2
CC            IF (A(K,K) .EQ. 0.0D0) INFO = K
CC   60 CONTINUE
CC      RETURN
CCC-----------------------------------------------------------------------
CCC The old factorization of A will be updated.  A row and a column
CCC has been added to the matrix A.
CCC N by N-1 is now the old size of the matrix.
CCC-----------------------------------------------------------------------
CC  70  CONTINUE
CC      NM1 = N - 1
CCC-----------------------------------------------------------------------
CCC Multiply the new column by the N previous Givens rotations.
CCC-----------------------------------------------------------------------
CC      DO 100 K = 1,NM1
CC        I = 2*(K-1) + 1
CC        T1 = A(K,N)
CC        T2 = A(K+1,N)
CC        C = Q(I)
CC        S = Q(I+1)
CC        A(K,N) = C*T1 - S*T2
CC        A(K+1,N) = S*T1 + C*T2
CC 100    CONTINUE
CCC-----------------------------------------------------------------------
CCC Complete update of decomposition by forming last Givens rotation,
CCC and multiplying it times the column vector (A(N,N),A(NP1,N)).
CCC-----------------------------------------------------------------------
CC      INFO = 0
CC      T1 = A(N,N)
CC      T2 = A(N+1,N)
CC      IF (T2 .NE. 0.0D0) GO TO 110
CC        C = 1.0D0
CC        S = 0.0D0
CC        GO TO 130
CC 110  CONTINUE
CC      IF (ABS(T2) .LT. ABS(T1)) GO TO 120
CC        T = T1/T2
CC        S = -1.0D0/SQRT(1.0D0+T*T)
CC        C = -S*T
CC        GO TO 130
CC 120  CONTINUE
CC        T = T2/T1
CC        C = 1.0D0/SQRT(1.0D0+T*T)
CC        S = -C*T
CC 130  CONTINUE
CC      IQ = 2*N - 1
CC      Q(IQ) = C
CC      Q(IQ+1) = S
CC      A(N,N) = C*T1 - S*T2
CC      IF (A(N,N) .EQ. 0.0D0) INFO = N
CC      RETURN
CCC
CCC------END OF SUBROUTINE DHEQR------------------------------------------
CC      END
CC      SUBROUTINE DHELS (A, LDA, N, Q, B)
CCC
CCC***BEGIN PROLOGUE  DHELS
CCC***DATE WRITTEN   890101   (YYMMDD)
CCC***REVISION DATE  900926   (YYMMDD)
CCC
CCC
CCC-----------------------------------------------------------------------
CCC***DESCRIPTION
CCC
CCC This is similar to the LINPACK routine DGESL except that
CCC A is an upper Hessenberg matrix.
CCC
CCC     DHELS solves the least squares problem
CCC
CCC           MIN (B-A*X,B-A*X)
CCC
CCC     using the factors computed by DHEQR.
CCC
CCC     On entry
CCC
CCC        A       DOUBLE PRECISION (LDA, N)
CCC                The output from DHEQR which contains the upper
CCC                triangular factor R in the QR decomposition of A.
CCC
CCC        LDA     INTEGER
CCC                The leading dimension of the array  A .
CCC
CCC        N       INTEGER
CCC                A is originally an (N+1) by N matrix.
CCC
CCC        Q       DOUBLE PRECISION(2*N)
CCC                The coefficients of the N givens rotations
CCC                used in the QR factorization of A.
CCC
CCC        B       DOUBLE PRECISION(N+1)
CCC                The right hand side vector.
CCC
CCC
CCC     On return
CCC
CCC        B       The solution vector X.
CCC
CCC
CCC     Modification of LINPACK.
CCC     Peter Brown, Lawrence Livermore Natl. Lab.
CCC
CCC-----------------------------------------------------------------------
CCC***ROUTINES CALLED
CCC   DAXPY 
CCC
CCC***END PROLOGUE  DHELS
CCC
CC      INTEGER LDA, N
CC      DOUBLE PRECISION A(LDA,*), B(*), Q(*)
CC      INTEGER IQ, K, KB, KP1
CC      DOUBLE PRECISION C, S, T, T1, T2
CCC
CCC        Minimize (B-A*X,B-A*X).
CCC        First form Q*B.
CCC
CC         DO 20 K = 1, N
CC            KP1 = K + 1
CC            IQ = 2*(K-1) + 1
CC            C = Q(IQ)
CC            S = Q(IQ+1)
CC            T1 = B(K)
CC            T2 = B(KP1)
CC            B(K) = C*T1 - S*T2
CC            B(KP1) = S*T1 + C*T2
CC   20    CONTINUE
CCC
CCC        Now solve R*X = Q*B.
CCC
CC         DO 40 KB = 1, N
CC            K = N + 1 - KB
CC            B(K) = B(K)/A(K,K)
CC            T = -B(K)
CC            CALL DAXPY (K-1, T, A(1,K), 1, B(1), 1)
CC   40    CONTINUE
CC      RETURN
CCC
CCC------END OF SUBROUTINE DHELS------------------------------------------
CC      END
C
CC  PROTOCOL OF MODIFICATIONS MADE BY ROBERT HUBER (HR)
CC  =====================================================      
CCLine 464: Documentation INFO(10)
CC40 replaced with 50
CCC                  IWORK(50+I) = +1 if Y(I) must be .GE. 0,
CCC                  IWORK(50+I) = +2 if Y(I) must be .GT. 0,
CCC                  IWORK(50+I) = -1 if Y(I) must be .LE. 0, while
CCC                  IWORK(50+I) = -2 if Y(I) must be .LT. 0, while
CCC                  IWORK(50+I) =  0 if Y(I) is not constrained.
CC
CCSubROUTINE DDASPK
CC
CC
CCLine 499: Documentation INFO(11)
CC40 replaced with 50
CCC                  IWORK(LID+I) = -1 if Y(I) is an algebraic variable,
CCC                  where LID = 50 if INFO(10) = 0 or 2 and LID = 50+NEQ
CCC                  if INFO(10) = 1 or 3.
CC
CCLine 586: Documentation INFO(16)
CCadded optin INFO(16)=2 
CCC HR 24.07.2008
CCC                set INFO(16) = 2 to scale components with stepsize H
CCC		                for stepsize control
CCC		 index 2 variables are scaled with H
CCC		index 3 variables are scaled wuth H^2
CCC	        The index 1,2,3 variables should appear in this order
CCC		The dimensions must be specified in IWORK(45), IWORK(46)
CCC		and IWORK(47)
CCC	        IWORK(45): dimension of index 1 variables (=NEQ for ODE)
CCC		IWORK(46): dimension of index 2 variables
CCC		IWORK(47): dimension of index 3 variables
CC
CCLine 608: Documentation INFO(16)
CC40 replaced with 50
CCC                       where LID = 50 if INFO(10) = 0 or 2 and 
CCC                       LID = 50 + NEQ if INFO(10) = 1 or 3.
CC
CCLine 761 and 766: Documentation LIW
CC40 replaced with 50
CCC             base = 50 + NEQ.
CCC             IF INFO(10) = 1 or 3, add NEQ to the base value.
CCC             If INFO(11) = 1 or INFO(16) =1, add NEQ to the base value.
CCC
CCC             If INFO(12) = 1 (Krylov method), the base value is
CCC             base = 50 + LENIWP.
CC
CCLine 1392: code
CCbasis of IWORK extended from 40 to 50 and LIND1, LIND2 and LIND3 added
CC      PARAMETER (LML=1, LMU=2, LMTYPE=4, 
CC     *   LIWM=1, LMXORD=3, LJCALC=5, LPHASE=6, LK=7, LKOLD=8,
CC     *   LNS=9, LNSTL=10, LNST=11, LNRE=12, LNJE=13, LETF=14, LNCFN=15,
CC     *   LNCFL=16, LNIW=17, LNRW=18, LNNI=19, LNLI=20, LNPS=21,
CC     *   LNPD=22, LMITER=23, LMAXL=24, LKMP=25, LNRMAX=26, LLNWP=27,
CC     *   LLNIWP=28, LLOCWP=29, LLCIWP=30, LKPRIN=31,
CC     *   LMXNIT=32, LMXNJ=33, LMXNH=34, LLSOFF=35, LIND1=45, LIND2=46,
CC     *   LIND3=47, LICNS=51)
CC
CCLine 1434: code
CCcheck of INFO(16) modified (.EG.2 now possible!)
CC     DO 15 I=12,15
CC         ITEMP = I
CC         IF (INFO(I) .NE. 0 .AND. INFO(I) .NE. 1) GO TO 701
CC 15      CONTINUE
CC      ITEMP = 16
CC      IF (INFO(16).LT.0 .OR. INFO(16).GT.2) GO TO 701
CC      ITEMP = 17
CC      IF (INFO(17) .NE. 0 .AND. INFO(17) .NE. 1) GO TO 701
CC
CCLine 1450: code
CCC     Check dimension of index 1,2,3 variables (HR 24.07.2008)
CC      SUMLIND = IWORK(LIND1)+IWORK(LIND2)+IWORK(LIND3)
CC      IF(SUMLIND .NE. NEQ) THEN
CC        WRITE(6,*)' SUM IWORK(45,46,47) .NE. NEQ'
CC        RETURN
CC      ENDIF
CC
CCLine 1540: code
CCC LINE BELOW: 24.07.2008 HR Changed from INFO(16).EQ. 1 to .NE. 0 
CC      IF (INFO(11) .EQ. 1 .OR. INFO(16) .NE. 1) LENID = NEQ
CC
CCLine 1572 and 1584: code
CC40 replaced with 50
CC         LENIW = 50 + LENIC + LENID + NEQ
CC...
CC         LENIW = 50 + LENIC + LENID + LENIWP
CC
CC
CCLine 1907: code
CCcomputation weight vector added
CCC    compute VT(H) 
CCC ADDED LINES BELOW:  HR 24.07.2008
CC      IF (INFO(16) .EQ. 2) THEN
CC        CALL DCALCVT(NEQ,H,INFO(2),RTOL,ATOL,Y,RWORK(LVT),IWORK(LIND1),
CC     *        IWORK(LIND2), IWORK(LIND3), RPAR, IPAR)
CC        CALL DINVWT(NEQ,RWORK(LVT),IER)
CC      IF (IER .NE. 0) GO TO 713
CC      ENDIF
CC
CC
CCLine 2103: code
CCcomputation weight vector added
CCC     Compute VT(H) 
CCC ADDED LINES BELOW:  HR 24.07.2008
CC      IF (INFO(16) .EQ. 2) THEN
CC        CALL DCALCVT(NEQ,H,INFO(2),RTOL,ATOL,Y,RWORK(LVT),IWORK(LIND1),
CC     *        IWORK(LIND2), IWORK(LIND3), RPAR, IPAR)
CC        CALL DINVWT(NEQ,RWORK(LVT),IER)
CC      IF (IER .NE. 0) GO TO 713
CC      ENDIF
CC
CCLine 2118: code
CCadditonal arguments in call of subroutine DDSTP
CC        CALL DDSTP(TN,Y,YPRIME,NEQ,
CC     *      RES,JAC,PSOL,H,RWORK(LWT),RWORK(LVT),INFO(1),IDID,RPAR,IPAR,
CC     *      RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
CC     *      RWORK(LWM),IWORK(LIWM),
CC     *      RWORK(LALPHA),RWORK(LBETA),RWORK(LGAMMA),
CC     *      RWORK(LPSI),RWORK(LSIGMA),
CC     *      RWORK(LCJ),RWORK(LCJOLD),RWORK(LHOLD),RWORK(LS),HMIN,
CC     *      RWORK(LROUND), RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
CC     *      RWORK(LEPCON), IWORK(LPHASE),IWORK(LJCALC),INFO(15),
CC     *      IWORK(LK), IWORK(LKOLD),IWORK(LNS),NONNEG,INFO(12),
CC     *      DNEDD,
CC     *      INFO(2), INFO(16), RTOL, ATOL, IWORK(LIND1), 
CC     *      IWORK(LIND2), IWORK(LIND3))
CC      ELSE IF (INFO(12) .EQ. 1) THEN
CC         CALL DDSTP(TN,Y,YPRIME,NEQ,
CC     *      RES,JAC,PSOL,H,RWORK(LWT),RWORK(LVT),INFO(1),IDID,RPAR,IPAR,
CC     *      RWORK(LPHI),RWORK(LSAVR),RWORK(LDELTA),RWORK(LE),
CC     *      RWORK(LWM),IWORK(LIWM),
CC     *      RWORK(LALPHA),RWORK(LBETA),RWORK(LGAMMA),
CC     *      RWORK(LPSI),RWORK(LSIGMA),
CC     *      RWORK(LCJ),RWORK(LCJOLD),RWORK(LHOLD),RWORK(LS),HMIN,
CC     *      RWORK(LROUND), RWORK(LEPLI),RWORK(LSQRN),RWORK(LRSQRN),
CC     *      RWORK(LEPCON), IWORK(LPHASE),IWORK(LJCALC),INFO(15),
CC     *      IWORK(LK), IWORK(LKOLD),IWORK(LNS),NONNEG,INFO(12),
CC     *      DNEDK,
CC     *      INFO(2), INFO(16), RTOL, ATOL, IWORK(LIND1), 
CC     *      IWORK(LIND2), IWORK(LIND3))
CC
CCLine 2427: code for error handling
CC727   MSG = 'DASPK--  Y(I) AND IWORK(50+I) (I=I1) INCONSISTENT'
CC
CC----------------------------------------------------------------------
CC
CCSUBROUTINE DDSTP
CC
CCLine 2663: arguments in definition of DDSTP
CCadded arguments INFO2,INFO16,RTOL,ATOL,NRIND1,NRIND2 and NRIND3
CC      SUBROUTINE DDSTP(X,Y,YPRIME,NEQ,RES,JAC,PSOL,H,WT,VT,
CC     *  JSTART,IDID,RPAR,IPAR,PHI,SAVR,DELTA,E,WM,IWM,
CC     *  ALPHA,BETA,GAMMA,PSI,SIGMA,CJ,CJOLD,HOLD,S,HMIN,UROUND,
CC     *  EPLI,SQRTN,RSQRTN,EPCON,IPHASE,JCALC,JFLG,K,KOLD,NS,NONNEG,
CC     *  NTYPE,NLS,INFO2,INFO16,RTOL,ATOL,NRIND1,NRIND2,NRIND3)
CC
CCLine 2675: documentation of DDSTP
CCC***REVISION DATE  080727   (YYMMDD) H dependened error test added (R. Huber)
CCC
CC
CC
CCLine 2738: code DDSTP
CCdeclaration fo RTOL and ATOL as arrays
CC      DIMENSION Y(*),YPRIME(*),WT(*),VT(*), RTOL(*), ATOL(*)
CC
CCLine 2878: code DDSTP
CCadded calculation VT
CCC    compute VT(H) 
CCC ADDED LINES BELOW:  HR 24.07.2008
CC      IF (INFO16 .EQ. 2) THEN
CC        CALL DCALCVT(NEQ,H,INFO2,RTOL,ATOL,Y,VT,NRIND1, NRIND2, NRIND3,
CC     *         RPAR, IPAR)
CC        CALL DINVWT(NEQ,VT,IER)
CC      IF (IER .NE. 0) THEN
CC        IDID = -2
CC        GO TO 675
CC      ENDIF
CC      ENDIF 
CC
CC-----------------------------------------------------------------------------
CCLine 3360: SUBROUTINE DCALCVT added!
CC
CC      SUBROUTINE DCALCVT(NEQ,HSCALE,IWT,RTOL,ATOL,Y,WT,NIND1,NIND2,
CC     *   NIND3,RPAR,IPAR)
CC  

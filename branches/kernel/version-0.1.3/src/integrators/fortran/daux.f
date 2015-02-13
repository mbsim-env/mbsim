*DECK D1MACH
      DOUBLE PRECISION FUNCTION D1MACH (IDUMMY)
C***BEGIN PROLOGUE  D1MACH
C***PURPOSE  Compute the unit roundoff of the machine.
C***CATEGORY  R1
C***TYPE      DOUBLE PRECISION (R1MACH-S, D1MACH-D)
C***KEYWORDS  MACHINE CONSTANTS
C***AUTHOR  Hindmarsh, Alan C., (LLNL)
C***DESCRIPTION
C *Usage:
C        DOUBLE PRECISION  A, D1MACH
C        A = D1MACH(idummy)  [The argument is ignored.]
C
C *Function Return Values:
C     A : the unit roundoff of the machine.
C
C *Description:
C     The unit roundoff is defined as the smallest positive machine
C     number u such that  1.0 + u .ne. 1.0.  This is computed by D1MACH
C     in a machine-independent manner.
C
C***REFERENCES  (NONE)
C***ROUTINES CALLED  DUMSUM
C***REVISION HISTORY  (YYYYMMDD)
C   19930216  DATE WRITTEN
C   19930818  Added SLATEC-format prologue.  (FNF)
C   20030707  Added DUMSUM to force normal storage of COMP.  (ACH)
C***END PROLOGUE  D1MACH
C
      INTEGER IDUMMY
      DOUBLE PRECISION U, COMP
C***FIRST EXECUTABLE STATEMENT  D1MACH
      U = 1.0D0
 10   U = U*0.5D0
      CALL DUMSUM(1.0D0, U, COMP)
      IF (COMP .NE. 1.0D0) GO TO 10
      D1MACH = U*2.0D0
      RETURN
C----------------------- End of Function D1MACH ------------------------
      END
C      SUBROUTINE DUMSUM(A,B,C)
CC     Routine to force normal storing of A + B, for D1MACH.
C      DOUBLE PRECISION A, B, C
C      C = A + B
C      RETURN
CC      END
C*DECK XERRWD
C      SUBROUTINE XERRWD (MSG, NMES, NERR, LEVEL, NI, I1, I2, NR, R1, R2)
CC***BEGIN PROLOGUE  XERRWD
CC***SUBSIDIARY
CC***PURPOSE  Write error message with values.
CC***LIBRARY   MATHLIB
CC***CATEGORY  R3C
CC***TYPE      DOUBLE PRECISION (XERRWV-S, XERRWD-D)
CC***AUTHOR  Hindmarsh, Alan C., (LLNL)
CC***DESCRIPTION
CC
CC  Subroutines XERRWD, XSETF, XSETUN, and the function routine IXSAV,
CC  as given here, constitute a simplified version of the SLATEC error
CC  handling package.
CC
CC  All arguments are input arguments.
CC
CC  MSG    = The message (character array).
CC  NMES   = The length of MSG (number of characters).
CC  NERR   = The error number (not used).
CC  LEVEL  = The error level..
CC           0 or 1 means recoverable (control returns to caller).
CC           2 means fatal (run is aborted--see note below).
CC  NI     = Number of integers (0, 1, or 2) to be printed with message.
CC  I1,I2  = Integers to be printed, depending on NI.
CC  NR     = Number of reals (0, 1, or 2) to be printed with message.
CC  R1,R2  = Reals to be printed, depending on NR.
CC
CC  Note..  this routine is machine-dependent and specialized for use
CC  in limited context, in the following ways..
CC  1. The argument MSG is assumed to be of type CHARACTER, and
CC     the message is printed with a format of (1X,A).
CC  2. The message is assumed to take only one line.
CC     Multi-line messages are generated by repeated calls.
CC  3. If LEVEL = 2, control passes to the statement   STOP
CC     to abort the run.  This statement may be machine-dependent.
CC  4. R1 and R2 are assumed to be in double precision and are printed
CC     in D21.13 format.
CC
CC***ROUTINES CALLED  IXSAV
CC***REVISION HISTORY  (YYMMDD)
CC   920831  DATE WRITTEN
CC   921118  Replaced MFLGSV/LUNSAV by IXSAV. (ACH)
CC   930329  Modified prologue to SLATEC format. (FNF)
CC   930407  Changed MSG from CHARACTER*1 array to variable. (FNF)
CC   930922  Minor cosmetic change. (FNF)
CC***END PROLOGUE  XERRWD
CC
CC*Internal Notes:
CC
CC For a different default logical unit number, IXSAV (or a subsidiary
CC routine that it calls) will need to be modified.
CC For a different run-abort command, change the statement following
CC statement 100 at the end.
CC-----------------------------------------------------------------------
CC Subroutines called by XERRWD.. None
CC Function routine called by XERRWD.. IXSAV
CC-----------------------------------------------------------------------
CC**End
CC
CC  Declare arguments.
CC
C      DOUBLE PRECISION R1, R2
C      INTEGER NMES, NERR, LEVEL, NI, I1, I2, NR
C      CHARACTER*(*) MSG
CC
CC  Declare local variables.
CC
C      INTEGER LUNIT, IXSAV, MESFLG
CC
CC  Get logical unit number and message print flag.
CC
CC***FIRST EXECUTABLE STATEMENT  XERRWD
C      LUNIT = IXSAV (1, 0, .FALSE.)
C      MESFLG = IXSAV (2, 0, .FALSE.)
C      IF (MESFLG .EQ. 0) GO TO 100
CC
CC  Write the message.
CC
C      WRITE (LUNIT,10)  MSG
C 10   FORMAT(1X,A)
C      IF (NI .EQ. 1) WRITE (LUNIT, 20) I1
C 20   FORMAT(6X,'In above message,  I1 =',I10)
C      IF (NI .EQ. 2) WRITE (LUNIT, 30) I1,I2
C 30   FORMAT(6X,'In above message,  I1 =',I10,3X,'I2 =',I10)
C      IF (NR .EQ. 1) WRITE (LUNIT, 40) R1
C 40   FORMAT(6X,'In above message,  R1 =',D21.13)
C      IF (NR .EQ. 2) WRITE (LUNIT, 50) R1,R2
C 50   FORMAT(6X,'In above,  R1 =',D21.13,3X,'R2 =',D21.13)
CC
CC  Abort the run if LEVEL = 2.
CC
C 100  IF (LEVEL .NE. 2) RETURN
C      STOP
CC----------------------- End of Subroutine XERRWD ----------------------
C      END
C*DECK XSETF
C      SUBROUTINE XSETF (MFLAG)
CC***BEGIN PROLOGUE  XSETF
CC***PURPOSE  Reset the error print control flag.
CC***LIBRARY   MATHLIB
CC***CATEGORY  R3A
CC***TYPE      ALL (XSETF-A)
CC***KEYWORDS  ERROR CONTROL
CC***AUTHOR  Hindmarsh, Alan C., (LLNL)
CC***DESCRIPTION
CC
CC   XSETF sets the error print control flag to MFLAG:
CC      MFLAG=1 means print all messages (the default).
CC      MFLAG=0 means no printing.
CC
CC***SEE ALSO  XERMSG, XERRWD, XERRWV
CC***REFERENCES  (NONE)
CC***ROUTINES CALLED  IXSAV
CC***REVISION HISTORY  (YYMMDD)
CC   921118  DATE WRITTEN
CC   930329  Added SLATEC format prologue. (FNF)
CC   930407  Corrected SEE ALSO section. (FNF)
CC   930922  Made user-callable, and other cosmetic changes. (FNF)
CC***END PROLOGUE  XSETF
CC
CC Subroutines called by XSETF.. None
CC Function routine called by XSETF.. IXSAV
CC-----------------------------------------------------------------------
CC**End
C      INTEGER MFLAG, JUNK, IXSAV
CC
CC***FIRST EXECUTABLE STATEMENT  XSETF
C      IF (MFLAG .EQ. 0 .OR. MFLAG .EQ. 1) JUNK = IXSAV (2,MFLAG,.TRUE.)
C      RETURN
CC----------------------- End of Subroutine XSETF -----------------------
C      END
C*DECK XSETUN
C      SUBROUTINE XSETUN (LUN)
CC***BEGIN PROLOGUE  XSETUN
CC***PURPOSE  Reset the logical unit number for error messages.
CC***LIBRARY   MATHLIB
CC***CATEGORY  R3B
CC***TYPE      ALL (XSETUN-A)
CC***KEYWORDS  ERROR CONTROL
CC***DESCRIPTION
CC
CC   XSETUN sets the logical unit number for error messages to LUN.
CC
CC***AUTHOR  Hindmarsh, Alan C., (LLNL)
CC***SEE ALSO  XERMSG, XERRWD, XERRWV
CC***REFERENCES  (NONE)
CC***ROUTINES CALLED  IXSAV
CC***REVISION HISTORY  (YYMMDD)
CC   921118  DATE WRITTEN
CC   930329  Added SLATEC format prologue. (FNF)
CC   930407  Corrected SEE ALSO section. (FNF)
CC   930922  Made user-callable, and other cosmetic changes. (FNF)
CC***END PROLOGUE  XSETUN
CC
CC Subroutines called by XSETUN.. None
CC Function routine called by XSETUN.. IXSAV
CC-----------------------------------------------------------------------
CC**End
C      INTEGER LUN, JUNK, IXSAV
CC
CC***FIRST EXECUTABLE STATEMENT  XSETUN
C      IF (LUN .GT. 0) JUNK = IXSAV (1,LUN,.TRUE.)
C      RETURN
CC----------------------- End of Subroutine XSETUN ----------------------
C      END
C*DECK IXSAV
C      INTEGER FUNCTION IXSAV (IPAR, IVALUE, ISET)
CC***BEGIN PROLOGUE  IXSAV
CC***SUBSIDIARY
CC***PURPOSE  Save and recall error message control parameters.
CC***LIBRARY   MATHLIB
CC***CATEGORY  R3C
CC***TYPE      ALL (IXSAV-A)
CC***AUTHOR  Hindmarsh, Alan C., (LLNL)
CC***DESCRIPTION
CC
CC  IXSAV saves and recalls one of two error message parameters:
CC    LUNIT, the logical unit number to which messages are printed, and
CC    MESFLG, the message print flag.
CC  This is a modification of the SLATEC library routine J4SAVE.
CC
CC  Saved local variables..
CC   LUNIT  = Logical unit number for messages.
CC   LUNDEF = Default logical unit number, data-loaded to 6 below
CC            (may be machine-dependent).
CC   MESFLG = Print control flag..
CC            1 means print all messages (the default).
CC            0 means no printing.
CC
CC  On input..
CC    IPAR   = Parameter indicator (1 for LUNIT, 2 for MESFLG).
CC    IVALUE = The value to be set for the parameter, if ISET = .TRUE.
CC    ISET   = Logical flag to indicate whether to read or write.
CC             If ISET = .TRUE., the parameter will be given
CC             the value IVALUE.  If ISET = .FALSE., the parameter
CC             will be unchanged, and IVALUE is a dummy argument.
CC
CC  On return..
CC    IXSAV = The (old) value of the parameter.
CC
CC***SEE ALSO  XERMSG, XERRWD, XERRWV
CC***ROUTINES CALLED  NONE
CC***REVISION HISTORY  (YYMMDD)
CC   921118  DATE WRITTEN
CC   930329  Modified prologue to SLATEC format. (FNF)
CC   941025  Minor modification re default unit number. (ACH)
CC***END PROLOGUE  IXSAV
CC
CC**End
C      LOGICAL ISET
C      INTEGER IPAR, IVALUE
CC-----------------------------------------------------------------------
C      INTEGER LUNIT, LUNDEF, MESFLG
CC-----------------------------------------------------------------------
CC The following Fortran-77 declaration is to cause the values of the
CC listed (local) variables to be saved between calls to this routine.
CC-----------------------------------------------------------------------
C      SAVE LUNIT, LUNDEF, MESFLG
C      DATA LUNIT/-1/, LUNDEF/6/, MESFLG/1/
CC
CC***FIRST EXECUTABLE STATEMENT  IXSAV
C      IF (IPAR .EQ. 1) THEN
C        IF (LUNIT .EQ. -1) LUNIT = LUNDEF
C        IXSAV = LUNIT
C        IF (ISET) LUNIT = IVALUE
C        ENDIF
CC
C      IF (IPAR .EQ. 2) THEN
C        IXSAV = MESFLG
C        IF (ISET) MESFLG = IVALUE
C        ENDIF
CC
C      RETURN
CC----------------------- End of Function IXSAV -------------------------
C      END
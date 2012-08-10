
set grid

set style line 1 lw 8
set style line 2 lw 8
set style line 3 lw 8
set style line 4 lw 8
set style line 5 lw 8

set logscale x
set logscale y

set xrange [2e2:2e5]
set yrange [1e-2:1e-1]

set xlabel "log(Re) [-]"
set ylabel "log(lambda) [-]"

set title "Widerstandszahl"

plot \
 "T100_kd0k0000_pl" u 2:3 w l title "k/d=0", \
 "T100_kd0k0050_pl" u 2:3 w l title "k/d=0.005", \
 "T100_kd0k0100_pl" u 2:3 w l title "k/d=0.01", \
 "T100_kd0k0150_pl" u 2:3 w l title "k/d=0.015", \
 "T100_kd0k0200_pl" u 2:3 w l title "k/d=0.02"
pause -1
set terminal postscript eps enhanced level1 color solid rounded size 7.4cm,5.25cm
set output "resistance_number.eps"
replot
set terminal x11
 
unset logscale x
unset logscale y
set xlabel "Q [l/min]"
set ylabel "pV [bar]"

set xrange [0:250]
set yrange [0:200]
set title "Druckverlust (global)"
plot \
 "T0_kd0k0000_pl" u 1:4 w l title "T=0", \
 "T25_kd0k0000_pl" u 1:4 w l title "T=25", \
 "T50_kd0k0000_pl" u 1:4 w l title "T=50", \
 "T75_kd0k0000_pl" u 1:4 w l title "T=75", \
 "T100_kd0k0000_pl" u 1:4 w l title "T=100"
pause -1
set terminal postscript eps enhanced level1 color solid rounded size 7.4cm,5.25cm
set output "pressureloss_global.eps"
replot
set terminal x11

set xrange [0:25]
set yrange [0:3]
set title "Druckverlust (Detail)";
replot
pause -1
set terminal postscript eps enhanced level1 color solid rounded size 7.4cm,5.25cm
set output "pressureloss_detail.eps"
replot
set terminal x11



#set title "T=0 [degC]"
#plot \
# "T0_kd0k0000_pl" u 1:4 w l title "k/d=0", \
# "T0_kd0k0050_pl" u 1:4 w l title "k/d=0.005", \
# "T0_kd0k0100_pl" u 1:4 w l title "k/d=0.01", \
# "T0_kd0k0150_pl" u 1:4 w l title "k/d=0.015", \
# "T0_kd0k0200_pl" u 1:4 w l title "k/d=0.02"
#pause -1
#
#set title "T=25 [degC]"
#plot \
# "T25_kd0k0000_pl" u 1:4 w l title "k/d=0", \
# "T25_kd0k0050_pl" u 1:4 w l title "k/d=0.005", \
# "T25_kd0k0100_pl" u 1:4 w l title "k/d=0.01", \
# "T25_kd0k0150_pl" u 1:4 w l title "k/d=0.015", \
# "T25_kd0k0200_pl" u 1:4 w l title "k/d=0.02"
#pause -1
#
#set title "T=50 [degC]"
#plot \
# "T50_kd0k0000_pl" u 1:4 w l title "k/d=0", \
# "T50_kd0k0050_pl" u 1:4 w l title "k/d=0.005", \
# "T50_kd0k0100_pl" u 1:4 w l title "k/d=0.01", \
# "T50_kd0k0150_pl" u 1:4 w l title "k/d=0.015", \
# "T50_kd0k0200_pl" u 1:4 w l title "k/d=0.02"
#pause -1
#
#set title "T=75 [degC]"
#plot \
# "T75_kd0k0000_pl" u 1:4 w l title "k/d=0", \
# "T75_kd0k0050_pl" u 1:4 w l title "k/d=0.005", \
# "T75_kd0k0100_pl" u 1:4 w l title "k/d=0.01", \
# "T75_kd0k0150_pl" u 1:4 w l title "k/d=0.015", \
# "T75_kd0k0200_pl" u 1:4 w l title "k/d=0.02"
#pause -1
#
#set title "T=100 [degC]"
#plot \
# "T100_kd0k0000_pl" u 1:4 w l title "k/d=0", \
# "T100_kd0k0050_pl" u 1:4 w l title "k/d=0.005", \
# "T100_kd0k0100_pl" u 1:4 w l title "k/d=0.01", \
# "T100_kd0k0150_pl" u 1:4 w l title "k/d=0.015", \
# "T100_kd0k0200_pl" u 1:4 w l title "k/d=0.02"
#pause -1

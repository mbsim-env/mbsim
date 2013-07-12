#!/bin/sh

make
./main 2> flowdata

octave -q evaluate_flowdata.m

gnuplot plot.gnuplot

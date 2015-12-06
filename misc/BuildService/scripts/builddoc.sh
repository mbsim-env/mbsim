#!/bin/sh

cd /home/mbsim/linux64-dailydebug/mbsim/misc/html/doc
CURDIR=$(pwd)

for MAINFILE in $(find -name main.tex); do
  cd $(dirname $MAINFILE)

  latex -halt-on-error -file-line-error main.tex &> latexout.txt
  ls *.bib &> /dev/null && bibtex main &>> latexout.txt
  latex -halt-on-error -file-line-error main.tex &>> latexout.txt
  latex -halt-on-error -file-line-error main.tex &>> latexout.txt
  latex -halt-on-error -file-line-error main.tex &>> latexout.txt
  dvips -o main.ps main.dvi &>> latexout.txt
  ps2pdf main.ps &>> latexout.txt

  cd $CURDIR
done

#!/bin/sh

if [ $# -ne 1 ]; then
  echo "Usage: $0 <doc-dir>"
  exit
fi

cd $1
CURDIR=$(pwd)

svn update

for MAINFILE in $(find -name main.tex); do
  cd $(dirname $MAINFILE)

  latex -halt-on-error -file-line-error main.tex &> latexout.txt
  latex -halt-on-error -file-line-error main.tex &>> latexout.txt
  latex -halt-on-error -file-line-error main.tex &>> latexout.txt
  dvips -o main.ps main.dvi &>> latexout.txt
  ps2pdf main.ps &>> latexout.txt

  cd $CURDIR
done

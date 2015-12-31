#!/usr/bin/python

import subprocess
import glob

subprocess.check_call(["pdflatex", "-halt-on-error", "-file-line-error", "main.tex"])
if len(glob.glob("*.bib"))>0:
  subprocess.check_call(["bibtex", "main"])
subprocess.check_call(["pdflatex", "-halt-on-error", "-file-line-error", "main.tex"])
subprocess.check_call(["pdflatex", "-halt-on-error", "-file-line-error", "main.tex"])
subprocess.check_call(["pdflatex", "-halt-on-error", "-file-line-error", "main.tex"])

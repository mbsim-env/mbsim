# LaTeX Makefile for dvi, ps, and pdf file creation.
#
# Usage:
# make          # make dvi, ps, and pdf
# make dvi      # make dvi
# make ps       # make ps (and dvi)
# make pdf      # make pdf
#

# Compiler
LATEX		= pdflatex --shell-escape --synctex=1
BIBTEX		= bibtex

# Sourcen
MAIN    	= main
PDF		= $(MAIN).pdf

all: pdf

pdf: ${MAIN}.pdf

${MAIN}.pdf : 
	@echo "----- Latex ------------------------------------------:"
	$(LATEX) ${MAIN}.tex
	$(LATEX) ${MAIN}.tex
	$(LATEX) ${MAIN}.tex

clean:
	rm -f ./$(MAIN).aux
	rm -f ./$(MAIN).bbl
	rm -f ./$(MAIN).blg
	rm -f ./$(MAIN).dvi
	rm -f ./$(MAIN).idx
	rm -f ./$(MAIN).log
	rm -f ./$(MAIN).out
	rm -f ./$(MAIN).pdf
	rm -f ./$(MAIN).spl
	rm -f ./$(MAIN).toc
	rm -f ./$(MAIN).synctex.gz
	rm -f ./*.pdf
	rm -f ./*.log
	rm -f ./*.aux

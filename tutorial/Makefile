# LaTeX Makefile for dvi, ps, and pdf file creation.
# By Jeffrey Humpherys
# Written April 05, 2004
# Revised January 13, 2005
# Thanks Bjorn and Boris
#
# Usage:
# make          # make dvi, ps, and pdf
# make dvi      # make dvi
# make ps       # make ps (and dvi)
# make pdf      # make pdf
#

# Compiler
LATEX		= latex
BIBTEX		= bibtex
# Sourcen
MAIN            = tutorial
DVI		= $(MAIN).dvi
PS		= $(MAIN).ps
PDF		= $(MAIN).pdf
SOURCES         = $(wildcard ./*.tex)
FIGUREDIR	= Figures
CODEDIR         = C++
EPSFIGURES      = $(wildcard ./$(FIGUREDIR)/*.eps)
OCTAVECODE      = $(wildcard ./$(CODEDIR)/*.c* ./$(CODEDIR)/*.h)

all: dvi ps pdf

dvi: ${MAIN}.dvi
pdf: ${MAIN}.pdf
ps: ${MAIN}.ps

${MAIN}.dvi : ${SOURCES} ${EPSFIGURES} ${OCTAVECODE}
	@echo "--- $(LATEX):"
	$(LATEX)  ${MAIN}

	@echo " "
	@echo "--- $(BIBTEX):"
	$(BIBTEX) ${MAIN}

	@echo " "
	@(while( grep -i "Rerun" ${MAIN}.log > /dev/null ) do \
		echo "--- Re-running $(ReRuns) $(LATEX):"; \
		$(LATEX) ${MAIN}  > /dev/null; \
	done)
	@echo " " \
# pipe to get him quite

	@echo " "
	@echo "--- $(LATEX): Most important warnings from $(MAIN).log:"
	@grep -i 'Warning\|Error\|multipl' $(MAIN).log
	@(test -f $(@:.dvi=.blg) \
	  && echo " " \
	  && echo "--- $(BIBTEX): Most important warnings from $(<@.dvi=.blg):" \
	  && grep -i 'Warning\|Error' $(@:.dvi=.blg)\
	  && echo " ")

${PS}  : ${DVI}
# running dvips
	dvips -ta4 ${DVI} -o $@

${PDF} : ${PS}
# running ps2pdf
	ps2pdf ${PS} $@ 

cleanall: clean
	rm -f $(DVI) ${PS} ${PDF}

clean:
	rm -f ./*.aux
	rm -f ./$(MAIN).toc
	rm -f ./$(MAIN).log
	rm -f ./$(MAIN).blg
	rm -f ./$(MAIN).dvi
	rm -f ./$(MAIN).pdf
	rm -f ./$(MAIN).ps
	rm -f ./*.tex~

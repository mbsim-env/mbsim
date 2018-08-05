# Compiler
LATEX	= pdflatex --shell-escape
BIBTEX 	= bibtex

# Sourcen
MAIN            = main
DVI				= $(MAIN).dvi
PS				= $(MAIN).ps
PDF				= $(MAIN).pdf
SOURCES         = $(wildcard ./*.tex)
EPSFIGURES      = $(wildcard ./*.eps)

all: dvi

dvi: ${MAIN}.dvi
pdf: ${MAIN}.pdf
ps: ${MAIN}.ps

${MAIN}.dvi : ${SOURCES} ${EPSFIGURES}
	@echo "--- $(LATEX):"
	$(LATEX) ${MAIN}
	$(BIBTEX) ${MAIN}
	$(LATEX) ${MAIN}
	$(BIBTEX) ${MAIN}
	$(LATEX) ${MAIN}

${PS}  : ${DVI}
	dvips -G0 -ta4 ${DVI} -o $@

${PDF} : ${PS}
	ps2pdf -dPDFSETTINGS=/prepress -dCompatibilityLevel=1.4 -dMAxSubsetPct=100 -dSubsetFonts=true -dEmbedAllFonts=true -sPAPERSIZE=a4 ${PS} $@ 

clean:
	rm -f ./*.aux
	rm -f ./*.log
	rm -f ./*.tex~
	rm -f ./$(MAIN).toc
	rm -f ./$(MAIN).lof
	rm -f ./$(MAIN).blg
	rm -f ./$(MAIN).out
	rm -f ./$(MAIN).bbl
	rm -f $(DVI) ${PS} ${PDF}

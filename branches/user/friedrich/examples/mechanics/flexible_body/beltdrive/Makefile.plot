# kill -USR2 <pid>
GNUPLOT=gnuplot
EPSTOEPS=eps2eps -dLanguageLevel=1
dsource=MBS.mbsim.h5
ddata = Disk0.h5dat Disk1.h5dat Disk2.h5dat TorqueGenerator.h5dat TensionerSpring.h5dat
#Belt.h5dat

sources=$(wildcard ./*.gpl)
figures=$(sources:.gpl=.eps)
copydir=/media/disk/BeltDrive/Paper/Figures/
#copydir=/home/home_dev/zander/Papers/2010_IMSD/Paper/Figures
#copydir=/winShare/Data/simpack/2010_IMSD/Paper/Figures
BASEPATH=$(shell basename $$PWD)
SUBDIRS=$(wildcard ???RPM*)

all: $(ddata) $(figures)

%.h5dat: $(dsource)
	h5dumpserie $</${@:.h5dat=}/data > $@

%.eps: $(ddata) %.gpl spck_belt.dat
	$(GNUPLOT) $(@:.eps=.gpl)
#	eps2eps $@ /tmp/$@
#	mv /tmp/$@ $@

copy: $(figures)
	cp $(figures) $(copydir)/$(BASEPATH)/
     

.PHONY : clean
clean:
	-rm $(figures) $(ddata)

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	$(MAKE) -C $@ copy


CC = g++
sources = main.cc system.cc 
#simulation_classes.cc modeling_classes.cc my_circuit.cc
libsources = 
objects = $(sources:.cc=.o) 
CPPFLAGS= -g3 `pkg-config --cflags mbsimElectronics` 





main : $(objects)
	$(CC) -o main $(objects) `pkg-config --libs mbsimElectronics` `pkg-config --libs mbsimControl`  




%.d: %.cc
	set -e; $(CC) -MM $(CPPFLAGS) $< \
                 | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	[ -s $@ ] || rm -f $@

include $(sources:.cc=.d)


.PHONY : clean
clean :
	-rm main $(objects) $(sources:.cc=.d)

clean_pos :
	-rm *.pos *.body

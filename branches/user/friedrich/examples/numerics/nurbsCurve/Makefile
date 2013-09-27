# Name of the Executable
Target = main

# Defining Sources
sources = main.cc

# Do Not Edit the Following Lines
CXX = g++

libsources = 
objects = $(sources:.cc=.o)

OPENMPFLAG = $(shell g++ -fopenmp -MM `pkg-config --cflags mbsimFlexibleBody` main.cc >& /dev/null && echo "-fopenmp")
CPPFLAGS= -g3 -Wall -Werror -Wno-unknown-pragmas $(OPENMPFLAG) `pkg-config --cflags mbsimFlexibleBody` 

$(Target) : $(objects)
	$(CXX) -o $@ $(objects) $(OPENMPFLAG) `pkg-config --libs mbsimFlexibleBody` 

%.d: %.cc
	set -e; $(CXX) -MM $(CPPFLAGS) $< \
	  | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	  [ -s $@ ] || rm -f $@

include $(sources:.cc=.d)

.PHONY : clean
clean :
	-rm $(Target) $(objects) $(sources:.cc=.d)

clean_pos :
	-rm *.pos *.body

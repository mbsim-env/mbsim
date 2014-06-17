# Name of the Executable
Target = main

# Defining Sources
sources = main.cc system.cc

# Do Not Edit the Following Lines
CXX = g++

libsources = 
objects = $(sources:.cc=.o) 

OPENMPFLAG = $(shell g++ -fopenmp -MM `pkg-config --cflags mbsimFlexibleBody` main.cc >& /dev/null && echo "-fopenmp")
CPPFLAGS= -g -O3 -Wall -Werror -Wno-unknown-pragmas $(OPENMPFLAG) `pkg-config --cflags mbsimFlexibleBody`
LDFLAGS=`pkg-config --libs-only-L mbsimFlexibleBody | sed 's/-L/-Wl,-rpath,/g'`
LIBFLAGS=`pkg-config --libs mbsimFlexibleBody`

$(Target) : $(objects)
	$(CXX) -o $@ $(objects) $(OPENMPFLAG) $(LDFLAGS) $(LIBFLAGS)

refresh : clean $(Target)

%.d: %.cc
	set -e; $(CXX) -MM $(CPPFLAGS) $< \
	  | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	  [ -s $@ ] || rm -f $@

include $(sources:.cc=.d)

.PHONY : clean
clean :
	-rm $(Target) $(objects) $(sources:.cc=.d)


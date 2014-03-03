# Name of the Executable
Target = main

# Defining Sources
sources = main.cc

# Do Not Edit the Following Lines
CXX = g++

libsources = 
objects = $(sources:.cc=.o)

OPENMPFLAG = $(shell g++ -fopenmp -MM `pkg-config --cflags mbsim` main.cc >& /dev/null && echo "-fopenmp")
CPPFLAGS= -g3 -O0 -Wall -Werror -Wno-unknown-pragmas $(OPENMPFLAG) `pkg-config --cflags mbsim`
LDFLAGS=`pkg-config --libs-only-L mbsim | sed 's/-L/-Wl,-rpath,/g'`
LIBFLAGS=`pkg-config --libs mbsim`

refresh : clean $(Target)

$(Target) : $(objects)
	$(CXX) -o $@ $(objects) $(OPENMPFLAG) $(LDFLAGS) $(LIBFLAGS)

%.d: %.cc
	set -e; $(CXX) -MM $(CPPFLAGS) $< \
	  | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	  [ -s $@ ] || rm -f $@

.PHONY : clean
clean :
	-rm $(Target) $(objects) $(sources:.cc=.d)

clean_pos :
	-rm *.pos *.body

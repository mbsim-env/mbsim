CC = g++
sources = main.cc system.cc
libsources = 
objects = $(sources:.cc=.o) 
CPPFLAGS= -g3 -Wall `pkg-config --cflags mbsim`
Target = woodpecker

$(Target) : $(objects) Makefile
	$(CC) -o $@ $(objects) `pkg-config --libs mbsim`

%.d: %.cc
	set -e; $(CC) -MM $(CPPFLAGS) $< \
                 | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	[ -s $@ ] || rm -f $@

include $(sources:.cc=.d)


.PHONY : clean
clean :
	-rm $(Target) $(objects) $(sources:.cc=.d)

CC = g++
sources = main.cc system.cc
libsources = 
objects = $(sources:.cc=.o) 
CPPFLAGS= -g -Wall `pkg-config --cflags mbsim` 



main : $(objects)
	$(CC) -o main $(objects) `pkg-config --libs mbsim`


%.d: %.cc
	set -e; $(CC) -MM $(CPPFLAGS) $< \
                 | sed 's/\(.*\)\.o[ :]*/\1.o \1.d : /g' > $@; \
	[ -s $@ ] || rm -f $@

include $(sources:.cc=.d)


.PHONY : clean
clean :
	-rm main $(objects) $(sources:.cc=.d)

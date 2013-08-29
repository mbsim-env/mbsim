sed -e '/#define\ /!d; s/#define\ /IPC\./g; s/\ /=/g; s/$/;/g' interfaceMessages.h > interfaceMessages.m

sed -e '/#define\ _SI_/!d; s/#define\ //g; s/\ .*//g; s/^.*$/case \0:\n  rIS << "\0";\n  break;/g' interfaceMessages.h > interfaceMessages.cc

// MBSim - Coding Standard
// =======================


// General
// -------


/*
 * - Use english names.
 *
 * - A class name starts with a uppercase character.
 *   The second and more words start with uppercase characters.
 *   Don't use underscores.
 * 
 * - A function name starts with a lowercase character.
 *   The second and more words start with uppercase characters.
 *   Don't use underscores.
 * 
 * - An indent consists of 2 spaces.
 *   (Use "set cindent shiftwidth=2" in .vimrc)
 * 
 * - Don't use tabs at all. So 4 indents are 8 spaces and NOT 1 tab.
 *   (Use "set expandtab" in .vimrc)
*/


// Code Style
// ----------


// Long comments:
/*
 * long comment
 * over more than one line
*/


// Short comments:
// short comment only one line


// Class:
class Foo : public Bar {
  public:
    void foo();
  protected:
    int bar(int i, double x); // some comment
    float inlineFoo() { return 5.3; }
    float inlineBar() {
      foo();
      return 5.2;
    }
};


// (Element) Funktion:
void Foo::foo() {
  bar();
}


// short if:
if(i==4) printf("test\n");
// or
if(i==4)
  printf("test\n");


// long if:
if(i==4) {
  printf("test1\n");
  printf("test2\n");
}


// short if else:
if(i==4)
  printf("test1\n");
else
  printf("test2\n");


// long if else:
if(i==4) {
  printf("test1a\n");
  printf("test1b\n");
}
else {
  printf("test2a\n");
  printf("test2b\n");
}


// short while:
while(i!=4) printf("test2a\n");
// or
while(i!=4)
  printf("test2a\n");


// long while:
while(i!=4) {
  printf("test1\n");
  printf("test2\n");
}


// short do-while:
do
  printf("test\n");
while(i==5);


// long do-while:
do {
  printf("test1\n");
  printf("test2\n");
}
while(i==5);


// switch-case:
switch(a) {
  case 1: printf("test1\n"); break
  case 2:
    printf("test2a\n");
    printf("test2b\n");
    break;
  default:
    printf("test3\n");
}


// Header file (filename myhead.h):
#ifndef MYHEAD_H
#define MYHEAD_H

// header code

#endif // MYHEAD_H


// Long preprocessor if:
#ifdef FOO
  foo1();
  foo2();
#else // not FOO
  bar1();
  bar2();
#endif // FOO

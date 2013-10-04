// HEADERS
#include "falcon/falcon.hpp"

// LIBRARY
#include <stdio.h>

// MAIN
int main(int argc, char** argv){

  // create falcon instance
  Falcon *falcon = new Falcon();

  // close falcon
  delete falcon;

  // print error message
  printf("Success\n");

  // exit program
  return 0;
}

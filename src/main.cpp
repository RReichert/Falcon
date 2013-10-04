// HEADERS
#include "falcon/falcon.hpp"

// LIBRARY
#include <iostream>
#include <cstdlib>

// MAIN
int main(int argc, char** argv){

  // create falcon instance
  Falcon falcon;

  // check if there are any
  if(falcon.hasError()) {
    cerr << "error: " << falcon.getError() << endl;
    return EXIT_FAILURE;
  } else {
    cout << "Success" << endl;
    return EXIT_SUCCESS;
  }
}

// HEADERS
#include "falcon/falcon.hpp"
#include "falcon/controller/pid.hpp"

// LIBRARY
#include <cstdlib>
#include <iostream>

// MAIN
int main(int argc, char** argv){

  // create falcon instance
  Falcon<Controller> falcon;

  // check if there are any
  if(falcon.hasError()) {
    cerr << "Error: " << falcon.getError() << endl;
    return EXIT_FAILURE;
  } else {
    cout << "Success" << endl;
    return EXIT_SUCCESS;
  }
}

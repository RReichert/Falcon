// HEADERS
#include "falcon/falcon.hpp"
#include "falcon/controller/test.hpp"

// LIBRARY
#include <cstdlib>
#include <iostream>

// MAIN
int main(int argc, char** argv){

  // create falcon instance
  Falcon<Test_Controller> falcon;

  // check if there are any
  if(falcon.isInit()) {
    cout << "Device is up" << endl;
  } else {
    cerr << "Error: " << falcon.getError() << endl;
    return EXIT_FAILURE;
  }

  // start controller
  falcon.start();

  // let the controller controll the device
  boost::this_thread::sleep(boost::posix_time::seconds(10));

  // exit successfully
  return EXIT_SUCCESS;
}

// HEADERS
#include "falcon/falcon.hpp"
#include "falcon/controller/test.hpp"

// LIBRARY
#include <cstdlib>
#include <iostream>
#include <boost/array.hpp>
#include <boost/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

// set debug filter
void init_debug() {
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
};

// MAIN
int main(int argc, char** argv){

  // setup debugger
  init_debug();

  // create falcon instance
  Falcon<Test_Controller> falcon;

  // check if falcon was initialized correctly
  if(!falcon.isInit()) {
    return EXIT_FAILURE;
  }

  // set desired tracking position
//  boost::array<double, 3> desiredPosition = {{0,0,0.16}};
//  falcon.setDesiredPosition(desiredPosition);

  // start controller
  falcon.start();

  // let the controller control the device
  boost::posix_time::ptime time;
  boost::array<double, 3> theta;
  boost::array<double, 3> omega;
  for(int x=0; x<100; x++) {
    falcon.getMotion(time, theta, omega);
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }

  // exit successfully
  return EXIT_SUCCESS;
}

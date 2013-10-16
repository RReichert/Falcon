// HEADERS
#include "falcon/falcon.hpp"
#include "falcon/controller/test.hpp"

// LIBRARY
#include <cstdlib>
#include <iostream>
#include <boost/array.hpp>
#include <boost/date_time.hpp>
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
    cout << "[" << time << "]:";
    cout << " Theta: [" << theta[0] << ", " << theta[1] << ", " << theta[2] << "]";
    cout << " Omega: [" << omega[0] << ", " << omega[1] << ", " << omega[2] << "]" << endl;
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }

  // exit successfully
  return EXIT_SUCCESS;
}

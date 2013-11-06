// HEADERS
#include "falcon/falcon.hpp"
#include "falcon/kinematics.hpp"
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
}

// MAIN
int main(int argc, char** argv){

  // setup debugger
  init_debug();

  // variables
  boost::posix_time::ptime time;
  boost::array<double, 3> theta;
  boost::array<double, 3> omega;

  boost::array<double, 9> thetas;
  boost::array<double, 3> position = {{0.02,-0.01,0.11}};
  boost::array<double, 3> estimatePosition = {{0,0,0.16}};

  // test kinematic algorithm
  bool status;
  Kinematics kinematic;

  BOOST_LOG_TRIVIAL(debug) << "start inverse kinematics";
  status = kinematic.inverse_kinematics(position, theta);
  BOOST_LOG_TRIVIAL(debug) << "inverse kinematics status: " << (status ? "success" : "failure");
  BOOST_LOG_TRIVIAL(debug) << "theta: " << "[" << theta[0] << ", "  << theta[1] << ", " << theta[2] << "]";

  BOOST_LOG_TRIVIAL(debug) << "start inverse kinematics";
  status = kinematic.inverse_kinematics(position, thetas);
  BOOST_LOG_TRIVIAL(debug) << "inverse kinematics status: " << (status ? "success" : "failure");
  BOOST_LOG_TRIVIAL(debug) << "thetas: " << "[" << thetas[0] << ", "  << thetas[1] << ", " << thetas[2] << ", "  << thetas[3] << ", "  << thetas[4] << ", " << thetas[5] << ", "  << thetas[6] << ", "  << thetas[7] << ", " << thetas[8] << "]";

  BOOST_LOG_TRIVIAL(debug) << "start forward kinematics";
  status = kinematic.forward_kinematics(theta, estimatePosition, position);
  BOOST_LOG_TRIVIAL(debug) << "forward kinematics status: " << (status ? "success" : "failure");
  BOOST_LOG_TRIVIAL(debug) << "position: " << "[" << position[0] << ", "  << position[1] << ", " << position[2] << "]";

  // create falcon instance
  Falcon<Test_Controller> falcon;

  // check if falcon was initialized correctly
  if(!falcon.isInit()) {
    return EXIT_FAILURE;
  }

  // set desired tracking position
  boost::array<double, 3> desiredPosition = {{0,0,0.16}};
  falcon.setDesiredPosition(desiredPosition);

  // start controller
  falcon.start();

  // let the controller control the device
  for(int x=0; x<100; x++) {
    falcon.getMotion(time, theta, omega);
    boost::this_thread::sleep(boost::posix_time::millisec(100));
  }

  // exit successfully
  return EXIT_SUCCESS;
}

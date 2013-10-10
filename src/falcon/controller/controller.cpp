#include "falcon/controller/controller.hpp"

Controller::~Controller() {

}

boost::array<double, 3> Controller::getForce(boost::array<double, 3> currentAngles, boost::array<double, 3> desiredAngles) {
  return boost::array<double, 3>();
}


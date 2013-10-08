#include "falcon/controller/controller.hpp"

Controller::Controller(boost::array<double, 3> startPosition) {
  this->position = startPosition;
}

boost::array<double, 3> Controller::getForce(boost::array<double, 3> currentPosition, boost::array<double, 3> desiredPosition) {
  return boost::array<double, 3>();
}


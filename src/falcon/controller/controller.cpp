#include "falcon/controller/controller.hpp"

Controller::~Controller() {

}

void Controller::getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque)) {
  torque[0] = 0.0;
  torque[1] = 0.0;
  torque[2] = 0.0;
}

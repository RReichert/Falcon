#include "falcon/controller/test.hpp"

void Test_Controller::getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque)) {
  torque[0] = 0.02;
  torque[1] = 0.02;
  torque[2] = 0.02;
}

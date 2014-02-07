#include "falcon/controller/test.hpp"

void Test_Controller::getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque)) {
  torque[0] = -1.2;
  torque[1] = -1.2;
  torque[2] = -1.2;
}

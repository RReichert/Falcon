#pragma once

#include <boost/array.hpp>
#include "falcon/controller/controller.hpp"

class Test_Controller : public Controller{

  public:

    void getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque));
};
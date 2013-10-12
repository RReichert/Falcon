#pragma once

#include "falcon/controller/controller.hpp"

#include <boost/array.hpp>

class Test_Controller : public Controller{

  public:

    // feedback controller method
    void getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque));
};

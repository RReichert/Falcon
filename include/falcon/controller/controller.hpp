#pragma once

#include <boost/array.hpp>

class Controller {

  public:

    virtual ~Controller();
    virtual void getTorque(const boost::array<double, 3> (&currentAngles), const boost::array<double, 3> (&desiredAngles), boost::array<double, 3> (&torque));
};

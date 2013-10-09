#pragma once

#include "falcon/controller/controller.hpp"

#include <boost/array.hpp>

class PID : public Controller{

  public:

    // feedback controller method
    boost::array<double, 3> getForce(boost::array<double, 3> currentAngles, boost::array<double, 3> desiredAngles);
};

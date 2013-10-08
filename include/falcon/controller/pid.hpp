#pragma once

#include "falcon/controller/controller.hpp"

#include <boost/array.hpp>

class PID : public Controller{

  public:

    // CONSTRUCTOR
    PID(boost::array<double, 3> startPosition);

    // feedback controller method
    boost::array<double, 3> getForce(boost::array<double, 3> currentPosition, boost::array<double, 3> desiredPosition);
};

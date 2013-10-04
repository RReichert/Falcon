#pragma once

#include "falcon/controller/controller.hpp"

#include <boost/array.hpp>

class PID {

  public:

    // CONSTRUCTOR
    PID(boost::array<double, 3> startPosition);
}

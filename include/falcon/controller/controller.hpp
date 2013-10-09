#pragma once

#include <boost/array.hpp>

class Controller {

  public:

    // DECONSTRUCTOR
    virtual ~Controller();

    // feedback controller method
    virtual boost::array<double, 3> getForce(boost::array<double, 3> currentAngles, boost::array<double, 3> desiredAngles);
};

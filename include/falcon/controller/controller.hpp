#pragma once

#include <boost/array.hpp>

class Controller {

  protected:

    // position variable
    boost::array<double, 3> position;

  public:

    // CONTROLLER
    Controller(boost::array<double, 3> startPosition);

    // feedback controller method
    virtual boost::array<double, 3> getForce(boost::array<double, 3> currentPosition, boost::array<double, 3> desiredPosition) = 0;
};

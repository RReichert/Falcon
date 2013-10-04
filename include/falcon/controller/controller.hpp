#pragma once

#include <boost/array.hpp>

class Falcon_Controller {

  protected:

    // position variable
    boost::array<double, 3> position; 

  public:

    // CONTROLLER
    Falcon_Controller(boost::array<double, 3> startPosition);
};

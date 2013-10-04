#pragma once

#include <boost/array.hpp>

class Falcon {

  protected:

    // positions
    boost::array<double, 3> actualPosition;
    boost::array<double, 3> desiredPosition;

  public:

    // CONSTRUCTOR
    Falcon();

    // starts controller
    void start();

    // stops controller
    void stop();

    // desired position accessor methods
    boost::array<double, 3> getActualPosition();
    boost::array<double, 3> getDesiredPosition();

    void setActualPosition(boost::array<double, 3> actualPosition);
    void setDesiredPosition(boost::array<double, 3> desiredPosition);

    // CALLBACK FUNCTION
    friend void falconCallback(Falcon&);
};

void falconCallback(Falcon&);

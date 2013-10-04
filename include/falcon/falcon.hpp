#pragma once

#include "falcon/controller/controller.hpp"

#include <iostream>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/static_assert.hpp>
#include <falcon/core/FalconDevice.h>

using namespace std;
using namespace libnifalcon;

template<typename Controller_Class>
class Falcon {

  // COMPILE TIME ASSERTIONS
  BOOST_STATIC_ASSERT((is_base_of<Controller, Controller_Class>::value));

  protected:

    // device
    FalconDevice device;

    // device's controller
    Controller_Class controller;

    // callback thread
    boost::thread *callbackThread;

    // positions
    boost::array<double, 3> actualPosition;
    boost::array<double, 3> desiredPosition;

  private:

    // error message
    string error;

  public:

    // CONSTRUCTOR
    Falcon();

    // DECONSTRUCTOR
    ~Falcon();

    // starts controller
    void start();

    // stops controller
    void stop();

    // error methods
    bool hasError();
    string getError();

    // desired position accessor methods
    boost::array<double, 3> getActualPosition();
    boost::array<double, 3> getDesiredPosition();

    void setActualPosition(boost::array<double, 3> actualPosition);
    void setDesiredPosition(boost::array<double, 3> desiredPosition);

    // CALLBACK FUNCTION
    friend void falconCallback(Falcon*);
};

void falconCallback(Falcon* falcon);

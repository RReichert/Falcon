#pragma once

#include "falcon/controller/controller.hpp"

#include <iostream>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <falcon/core/FalconDevice.h>

using namespace std;
using namespace libnifalcon;

template<class T>
class Falcon {

  // COMPILE TIME ASSERTIONS
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  protected:

    // device
    FalconDevice device;

    // device's controller
    Controller *controller;

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
    friend void falconCallback(Falcon<T>*);
};

template<class T> Falcon<T>::Falcon() {
/*
  // check how many devices are connected
  unsigned int deviceCount;
  device.getDeviceCount(deviceCount);

  // report if no falcon was found
  if(deviceCount == 0) {
    error = "no device found";
    return;
  }

  // attempt to open up connection with device
  for(unsigned int x=0; x<deviceCount; x++) {

    // attempt to open up connection
    if(!device.open(x)) {
      continue;
    }
  }

  // report on connection status
  if(!device.isOpen()) {
    error = "unable to communicate with any of the devices";
    return;
  }
*/
  // setup controller
  //controller = (Controller) new Ctrl();

  // create callback function thread
//  callbackThread = new boost::thread(falconCallback, this);
}

template<class T> Falcon<T>::~Falcon() {

  // close the thread
  if(!callbackThread) {
    callbackThread->join();
  }

  // close the falcon communication
  device.close();

}

template<class T> bool Falcon<T>::hasError() {
  return !error.empty();
}

template<class T> string Falcon<T>::getError() {
  return error;
}

template<class T> void falconCallback(Falcon<T> *falcon) {
}


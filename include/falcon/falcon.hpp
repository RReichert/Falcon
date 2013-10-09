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

  // ASSERT THAT THE CLASS BEING PASSED IS A CHILD OF CONTROLLER 
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  private:

    // state
    bool initialized;

    // callback thread
    boost::thread *callbackThread;

    // error message
    string error;

  protected:

    // device
    FalconDevice device;
    boost::shared_ptr<FalconFirmware> firmware;
    boost::shared_ptr<FalconKinematic> kinematic;

    // controller
    Controller *controller;

  public:

    // CONSTRUCTOR
    Falcon();

    // DECONSTRUCTOR
    ~Falcon();

    // initializer
    bool init();

    // un-initialize
    void uninit();

    // initialized?
    bool isInit();

    // starts controller
    void start();

    // stops controller
    void stop();

    // error methods
    bool hasError();
    string getError();

    // position methods 
    boost::array<double, 3> getCurrentPosition();
    void setDesiredPosition(boost::array<double, 3> desiredPosition);

    // CALLBACK FUNCTION
    friend void falconCallback(Falcon<T>*);
};

template<class T>
void falconCallback(Falcon<T> *falcon) {

}

template<class T>
Falcon<T>::Falcon() : initialized(false) {

  // initialize the device
  init();

}

template<class T>
bool Falcon<T>::init() {

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

  // obtain firmware & kinematic
  firmware = device.getFalconFirmware();
  kinematic = device.getFalconKinematic();

  // setup controller
  controller = (Controller*) new T();

  // create callback function thread
  callbackThread = new boost::thread(falconCallback<T>, this);

  // return state
  return initialized;
}

template<class T>
Falcon<T>::~Falcon() {

  // close the thread
  if(!callbackThread) {
    callbackThread->join();
  }

  // close the falcon communication
  device.close();

}

template<class T>
bool Falcon<T>::hasError() {
  return !error.empty();
}

template<class T>
string Falcon<T>::getError() {
  return error;
}

template<class T>
boost::array<double, 3> getCurrentPosition(){
  if(initialized) {
    
  } else {

  }
  return boost::array<double, 3>();
}

template<class T>
void setDesiredPosition(boost::array<double, 3> desiredPosition){
  return;
}

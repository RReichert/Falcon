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

template<class T=Controller>
class Falcon {

  // ASSERT THAT THE CLASS BEING PASSED IS A CHILD OF CONTROLLER 
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  private:

    // callback thread
    boost::thread *callbackThread;

    // error message
    string error;

  protected:

    // state
    bool initialized;
    bool running;

    // device
    FalconDevice device;
    boost::shared_ptr<FalconFirmware> firmware;
    boost::shared_ptr<FalconKinematic> kinematic;

    // controller
    Controller *controller;

    // falcon-controller
    bool hasDesiredAngles;
    boost::array<double, 3> desiredAngles;

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

    // running?
    bool isRunning();

    // error methods
    bool hasError();
    string getError();

    // position methods 
    bool getCurrentAngles(boost::array<double, 3> &currentAngles);
    bool getCurrentPosition(boost::array<double, 3> &currentPosition);
    void setDesiredPosition(boost::array<double, 3> desiredPosition);

    // CALLBACK FUNCTION
    template<class X>
    friend void falconCallback(Falcon<X>*);
};

template<class T>
void falconCallback(Falcon<T> *falcon) {

  // function variables
  boost::array<int, 3> forceInt;
  boost::array<double, 3> force;
  boost::array<double, 3> angles;
  const boost::array<double, 3> zeros = {{0,0,0}}; 

  // while device falcon is initialized
  while(falcon->initialized) {

    // if device has not requested a controller - wait 
    if(!falcon->running) {
      // wait till running is set
      continue;
    }

    // reset force variable
    force = zeros;

    // if a valid desired angle was provided
    if(falcon->hasDesiredAngles) {
      force = falcon->controller->getForce(angles, falcon->desiredAngles);

    }

    // convert torque unit to falcon 

    // set feedback force
    falcon->firmware->setForces(forceInt);
  }

}

template<class T>
Falcon<T>::Falcon() : running(false), hasDesiredAngles(false) {
  init();
}

template<class T>
Falcon<T>::~Falcon() {
  uninit();
}

template<class T>
bool Falcon<T>::init() {

  // clear any error message
  error.clear();

  try{

    // check how many devices are connected
    unsigned int deviceCount;
    device.getDeviceCount(deviceCount);

    // check number of falcon's connected 
    if(deviceCount == 0) {
      throw "no device found";
    }

    // attempt to open up connection with device
    for(unsigned int x=0; x<deviceCount; x++) {
      if(!device.open(x)) {
        continue;
      }
    }

    // report on connection status
    if(!device.isOpen()) {
      throw "unable to communicate with any of the devices";
    }

    // obtain firmware & kinematic
    firmware = device.getFalconFirmware();
    kinematic = device.getFalconKinematic();

    // setup controller
    controller = (Controller*) new T();

    // create callback function thread
    callbackThread = new boost::thread(falconCallback<T>, this);

    // successfully initialized
    initialized = true;

  } catch(char const* msg) {

    // clean up if error uninitialized 
    uninit();

    // record error message
    error = msg;
  }

  // return initialized results
  return initialized;
}

template<class T>
void Falcon<T>::uninit() {

  // flag as uninitialized
  running = false;
  initialized = false;

  // close the thread
  if(callbackThread) {
    callbackThread->join();
    delete callbackThread;
  }

  // destroy controller
  if(controller) {
    delete controller;
  }

  // close the falcon communication
  device.close();

}

template<class T>
bool Falcon<T>::isInit() {
  return initialized;
}

template<class T>
void Falcon<T>::start() {
  running = true;
}

template<class T>
void Falcon<T>::stop() {
  running = false;
}

template<class T>
bool Falcon<T>::isRunning() {
  return running;
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
bool Falcon<T>::getCurrentAngles(boost::array<double, 3> &currentAngles) {
  if(initialized) {
    kinematic->getAngles(device.getPosition(), currentAngles);
  }

  return initialized;
}

template<class T>
bool Falcon<T>::getCurrentPosition(boost::array<double, 3> &currentPosition) {
  if(initialized) {
    currentPosition = device.getPosition();
  }

  return initialized;
}

template<class T>
void Falcon<T>::setDesiredPosition(boost::array<double, 3> desiredPosition){
  hasDesiredAngles = kinematic->getAngles(desiredPosition, desiredAngles);
}

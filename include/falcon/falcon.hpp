#pragma once

#include "falcon/controller/controller.hpp"

#include <iostream>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/type_traits.hpp>
#include <boost/static_assert.hpp>
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

using namespace std;
using namespace libnifalcon;

template<class T=Controller>
class Falcon {

  // ASSERT THAT THE CLASS BEING PASSED IS A CHILD OF CONTROLLER 
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  private:

    // state
    bool running = false;
    bool initialized = false;

    // interthread channel
    boost::mutex runningMutex;
    boost::condition_variable runningEvent;

    // device
    FalconDevice device;
    boost::shared_ptr<FalconFirmware> firmware;
    boost::shared_ptr<FalconKinematic> kinematic;

    // controller
    Controller *controller = NULL;

    // falcon-controller
    bool hasDesiredAngles = NULL;
    boost::array<double, 3> desiredAngles;

    // callback thread
    boost::thread *callbackThread = NULL;

    // error message
    string error;

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
    void setDesiredPosition(boost::array<double, 3> &desiredPosition);

    // CALLBACK FUNCTION
    void operator() (); 
};

template<class T>
void Falcon<T>::operator() () {

  // establish scoped lock on running event
  boost::unique_lock<boost::mutex> lock(runningMutex); 

  // function variables
  boost::array<double, 3> angles;
  boost::array<double, 3> torque;
  boost::array<int, 3> encodedTorque;
  const boost::array<double, 3> zeros = {{0,0,0}}; 

  // while device falcon is initialized
  while(initialized) {

    // if device has not requested a controller - wait 
    if(!running) {
      runningEvent.wait(lock);
      continue;
    }

    // MAIN DEVICE LOOP 
    device.runIOLoop();

    // DO NOT INTERRUPT
    {
      // interruption marker
      boost::this_thread::disable_interruption iPoint; 

      // reset torque variable
      torque = zeros;

      // if a valid desired angle was provided
      if(hasDesiredAngles) {
        torque = controller->getTorque(angles, desiredAngles);
      }

      // convert torque to motor voltages:
      encodedTorque[0] = -10000.0*torque[0];
      encodedTorque[1] = -10000.0*torque[1];
      encodedTorque[2] = -10000.0*torque[2];

      // NOTE: this is from the libnifalcon's FalconKinematicStamper.cpp

      // set feedback torque 
      firmware->setForces(encodedTorque);
    }
  }

}

template<class T>
Falcon<T>::Falcon() {

  // set firmware & kinematic
  device.setFalconFirmware<FalconFirmwareNovintSDK>();
  device.setFalconKinematic<FalconKinematicStamper>();

  // obtain firmware & kinematic
  firmware = device.getFalconFirmware();
  kinematic = device.getFalconKinematic();

  // initialize device
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

    // error if not devices were found 
    if(deviceCount == 0) {
      throw "no device found";
    }

    // attempt to open up connection with device
    for(unsigned int x=0; x<deviceCount; x++) {
      if(!device.open(x)) {
        continue;
      }
    }

    // report failure to connect with device
    if(!device.isOpen()) {
      throw "unable to communicate with any of the devices";
    }

    // check firmware is initialized
    bool firmwareLoaded = device.isFirmwareLoaded();
    if(!firmwareLoaded) {

      // firmware property variables
      bool skip_checksum = false;
      long firmware_size = NOVINT_FALCON_NVENT_FIRMWARE_SIZE;
      uint8_t* firmware_block = const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE);

      // attempt to load firmware a few times
      for(int i = 0; i < 10; i++) {
        if(firmware->loadFirmware(skip_checksum, firmware_size, firmware_block)) {
          firmwareLoaded = true;
          break;
        }
      }
    }

    // report if firmware was not loaded 
    if(!firmwareLoaded) {
      throw "unable to load firmware";
    }

    // startup controller
    controller = (Controller*) new T();

    // thread off callback function 
    callbackThread = new boost::thread(boost::ref(*this));

    // flag successful initialization
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
  initialized = false;

  // flag to stop controller
  stop();

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
  runningEvent.notify_all();
}

template<class T>
void Falcon<T>::stop() {
  running = false;
  runningEvent.notify_all();
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
    boost::array<double, 3> currentPosition = device.getPosition();
    kinematic->getAngles(currentPosition, currentAngles);
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
void Falcon<T>::setDesiredPosition(boost::array<double, 3> &desiredPosition){
  hasDesiredAngles = kinematic->getAngles(desiredPosition, desiredAngles);
}

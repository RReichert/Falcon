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

  // ASSERT THAT THE CLASS BEING PASSED IS A CHILD OF CONTROLLER CLASS
  BOOST_STATIC_ASSERT((boost::is_base_of<Controller, T>::type::value));

  private:

    // state
    bool running;
    bool initialized;

    // device
    FalconDevice device;
    boost::shared_ptr<FalconFirmware> firmware;
    boost::shared_ptr<FalconKinematic> kinematic;

    // controller
    Controller *controller;

    // falcon-controller shared resource
    boost::mutex thetaMutex;
    boost::array<double, 3> theta;

    boost::mutex omegaMutex;
    boost::array<double, 3> omega;
    
    boost::mutex desiredThetaMutex;
    boost::array<double, 3> desiredTheta;

    // callback thread
    boost::thread *callbackThread;

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

    // public sharing resource 
    bool setDesiredPosition(boost::array<double, 3> (&desiredPosition));

    bool getTheta(boost::array<double, 3> (&theta));
    bool getOmega(boost::array<double, 3> (&omega));

    bool getPosition(boost::array<double, 3> (&position));
    bool getVelocity(boost::array<double, 3> (&velocity));

    // NOTE: for each of the methods above, if the system is not initialized
    //       or there is an error, false is returned. only exception is
    //       setDesiredPosition, where if the position is outside the workspace
    //       the value will not be set and function will return false. if all 
    //       is well, true is returned

    // CALLBACK FUNCTION
    void operator() (); 

  private:

    // private sharing resource
    void setTheta(boost::array<double, 3> (&theta));
    void setOmega(boost::array<double, 3> (&omega));
    void getDesiredTheta(boost::array<double, 3> (&desiredTheta));

};

template<class T>
void Falcon<T>::operator() () {

  // function variables
  boost::array<double, 3> theta;
  boost::array<double, 3> torque;
  boost::array<int, 3> encodedTorque;
  const boost::array<double, 3> zeros = {{0,0,0}}; 

  // while device falcon is initialized
  while(initialized) {

    // MAIN DEVICE LOOP 
    device.runIOLoop();

    // DO NOT INTERRUPT
    {
      // interruption marker
      boost::this_thread::disable_interruption iPoint; 

      // CALCULATE POSITION AND VELOCITY

      // if controller is not running
      if(!running) {
        continue;
      }

      // reset torque and get desired theta
      torque = zeros;
      getDesiredTheta(desiredTheta);

      // call the controller 
      controller->getTorque(theta, desiredTheta, torque);

      // convert torque to motor voltages:
      encodedTorque[0] = torque[0];
      encodedTorque[1] = torque[1];
      encodedTorque[2] = torque[2];

      // NOTE: this will require the formula from Characteristic of the Novint Falcon

      // set feedback torque 
      firmware->setForces(encodedTorque);
    }
  }

}

template<class T>
Falcon<T>::Falcon() :
  running(false),
  initialized(false),
  controller(NULL),
  callbackThread(NULL) {

  // set default desired theta value
  desiredTheta = {{0,0,0.11}};

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

    // create callback function 
    callbackThread = new boost::thread(boost::ref(*this));

    // check if callback thread was created
    if(!callbackThread) {
      throw "unable to spawn callback thread";
    }

    // flag successful initialization
    initialized = true;

  } catch(char const* msg) {

    // uninitialized device
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
void Falcon<T>::getDesiredTheta(boost::array<double, 3> (&desiredTheta)) {
  boost::lock_guard<boost::mutex> lock(desiredThetaMutex);
  desiredTheta = this->desiredTheta;
}

template<class T>
bool Falcon<T>::setDesiredPosition(boost::array<double, 3> (&desiredPosition)) {
  boost::lock_guard<boost::mutex> lock(desiredThetaMutex);
  return kinematic->getAngles(desiredPosition, desiredTheta);
}

template<class T>
bool Falcon<T>::getTheta(boost::array<double, 3> (&theta)) {
  if(!initialized) {
    return false;
  }
  
  boost::lock_guard<boost::mutex> lock(thetaMutex);
  theta = this->theta;
  return true;  
}

template<class T>
void Falcon<T>::setTheta(boost::array<double, 3> (&theta)) {
  boost::lock_guard<boost::mutex> lock(thetaMutex);
  this->theta = theta;
}

template<class T>
bool Falcon<T>::getOmega(boost::array<double, 3> (&omega)) {
  if(!initialized) {
    return false;
  }
  
  boost::lock_guard<boost::mutex> lock(omegaMutex);
  omega = this->omega;
  return true;  
}

template<class T>
void Falcon<T>::setOmega(boost::array<double, 3> (&omega)) {
  boost::lock_guard<boost::mutex> lock(omegaMutex);
  this->omega = omega;
}

template<class T>
bool Falcon<T>::getPosition(boost::array<double, 3> (&position)) {
  if(!initialized) {
    return false;
  }

  // lock the theta while we make a local copy of it  
  boost::array<double, 3> theta;
  thetaMutex.lock();
  theta = this->theta;
  thetaMutex.unlock();  

  // CALCULATE THE POSITION USING theta
  return true;  
}

template<class T>
bool Falcon<T>::getVelocity(boost::array<double, 3> (&velocity)) {
  if(!initialized) {
    return false;
  }

  // lock the theta while we make a local copy of it  
  boost::array<double, 3> omega;
  omegaMutex.lock();
  omega = this->omega;
  omegaMutex.unlock();  

  // CALCULATE THE VELOCITY USING omega
  return true;  
}
